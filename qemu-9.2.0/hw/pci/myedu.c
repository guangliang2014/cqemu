/*
 * QEMU MYEDU PCIe device (based on edu.c)
 *
 * Minimal skeletal device derived from hw/misc/edu.c for educational use.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"

#define TYPE_PCI_MYEDU_DEVICE "myedu"
typedef struct MyEduState MyEduState;
DECLARE_INSTANCE_CHECKER(MyEduState, MYEDU,
                         TYPE_PCI_MYEDU_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

struct MyEduState {
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define MYEDU_STATUS_COMPUTING    0x01
#define MYEDU_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define MYEDU_DMA_RUN             0x1
#define MYEDU_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define MYEDU_DMA_FROM_PCI       0
# define MYEDU_DMA_TO_PCI         1
#define MYEDU_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
};

static bool myedu_msi_enabled(MyEduState *edu)
{
    return msi_enabled(&edu->pdev);
}

static void myedu_raise_irq(MyEduState *edu, uint32_t val)
{
    edu->irq_status |= val;
    if (edu->irq_status) {
        if (myedu_msi_enabled(edu)) {
            msi_notify(&edu->pdev, 0);
        } else {
            pci_set_irq(&edu->pdev, 1);
        }
    }
}

static void myedu_lower_irq(MyEduState *edu, uint32_t val)
{
    edu->irq_status &= ~val;

    if (!edu->irq_status && !myedu_msi_enabled(edu)) {
        pci_set_irq(&edu->pdev, 0);
    }
}

static void myedu_check_range(uint64_t xfer_start, uint64_t xfer_size,
                uint64_t dma_start, uint64_t dma_size)
{
    uint64_t xfer_end = xfer_start + xfer_size;
    uint64_t dma_end = dma_start + dma_size;

    if (dma_end >= dma_start && xfer_end >= xfer_start &&
        xfer_start >= dma_start && xfer_end <= dma_end) {
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "MYEDU: DMA range 0x%016"PRIx64"-0x%016"PRIx64
                  " out of bounds (0x%016"PRIx64"-0x%016"PRIx64")!",
                  xfer_start, xfer_end - 1, dma_start, dma_end - 1);
}

static dma_addr_t myedu_clamp_addr(const MyEduState *edu, dma_addr_t addr)
{
    dma_addr_t res = addr & edu->dma_mask;

    if (addr != res) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "MYEDU: clamping DMA 0x%016"PRIx64" to 0x%016"PRIx64"!",
                      addr, res);
    }

    return res;
}

static void myedu_dma_timer(void *opaque)
{
    MyEduState *edu = opaque;
    bool raise_irq = false;

    if (!(edu->dma.cmd & MYEDU_DMA_RUN)) {
        return;
    }

    if (MYEDU_DMA_DIR(edu->dma.cmd) == MYEDU_DMA_FROM_PCI) {
        uint64_t dst = edu->dma.dst;
        myedu_check_range(dst, edu->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&edu->pdev, myedu_clamp_addr(edu, edu->dma.src),
                edu->dma_buf + dst, edu->dma.cnt);
    } else {
        uint64_t src = edu->dma.src;
        myedu_check_range(src, edu->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&edu->pdev, myedu_clamp_addr(edu, edu->dma.dst),
                edu->dma_buf + src, edu->dma.cnt);
    }

    edu->dma.cmd &= ~MYEDU_DMA_RUN;
    if (edu->dma.cmd & MYEDU_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        myedu_raise_irq(edu, DMA_IRQ);
    }
}

static void dma_rw_myedu(MyEduState *edu, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (edu->dma.cmd & MYEDU_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&edu->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static uint64_t myedu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MyEduState *edu = opaque;
    uint64_t val = ~0ULL;

    if (addr < 0x80 && size != 4) {
        return val;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return val;
    }

    switch (addr) {
    case 0x00:
        val = 0x010000edu;
        break;
    case 0x04:
        val = edu->addr4;
        break;
    case 0x08:
        qemu_mutex_lock(&edu->thr_mutex);
        val = edu->fact;
        qemu_mutex_unlock(&edu->thr_mutex);
        break;
    case 0x20:
        val = qatomic_read(&edu->status);
        break;
    case 0x24:
        val = edu->irq_status;
        break;
    case 0x80:
        dma_rw_myedu(edu, false, &val, &edu->dma.src, false);
        break;
    case 0x88:
        dma_rw_myedu(edu, false, &val, &edu->dma.dst, false);
        break;
    case 0x90:
        dma_rw_myedu(edu, false, &val, &edu->dma.cnt, false);
        break;
    case 0x98:
        dma_rw_myedu(edu, false, &val, &edu->dma.cmd, false);
        break;
    }

    return val;
}

static void myedu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    MyEduState *edu = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return;
    }

    switch (addr) {
    case 0x04:
        edu->addr4 = ~val;
        break;
    case 0x08:
        if (qatomic_read(&edu->status) & MYEDU_STATUS_COMPUTING) {
            break;
        }
        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = val;
        qatomic_or(&edu->status, MYEDU_STATUS_COMPUTING);
        qemu_cond_signal(&edu->thr_cond);
        qemu_mutex_unlock(&edu->thr_mutex);
        break;
    case 0x20:
        if (val & MYEDU_STATUS_IRQFACT) {
            qatomic_or(&edu->status, MYEDU_STATUS_IRQFACT);
            smp_mb__after_rmw();
        } else {
            qatomic_and(&edu->status, ~MYEDU_STATUS_IRQFACT);
        }
        break;
    case 0x60:
        myedu_raise_irq(edu, val);
        break;
    case 0x64:
        myedu_lower_irq(edu, val);
        break;
    case 0x80:
        dma_rw_myedu(edu, true, &val, &edu->dma.src, false);
        break;
    case 0x88:
        dma_rw_myedu(edu, true, &val, &edu->dma.dst, false);
        break;
    case 0x90:
        dma_rw_myedu(edu, true, &val, &edu->dma.cnt, false);
        break;
    case 0x98:
        if (!(val & MYEDU_DMA_RUN)) {
            break;
        }
        dma_rw_myedu(edu, true, &val, &edu->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps myedu_mmio_ops = {
    .read = myedu_mmio_read,
    .write = myedu_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },

};

static void *myedu_fact_thread(void *opaque)
{
    MyEduState *edu = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&edu->thr_mutex);
        while ((qatomic_read(&edu->status) & MYEDU_STATUS_COMPUTING) == 0 &&
                        !edu->stopping) {
            qemu_cond_wait(&edu->thr_cond, &edu->thr_mutex);
        }

        if (edu->stopping) {
            qemu_mutex_unlock(&edu->thr_mutex);
            break;
        }

        val = edu->fact;
        qemu_mutex_unlock(&edu->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = ret;
        qemu_mutex_unlock(&edu->thr_mutex);
        qatomic_and(&edu->status, ~MYEDU_STATUS_COMPUTING);

        smp_mb__after_rmw();

        if (qatomic_read(&edu->status) & MYEDU_STATUS_IRQFACT) {
            bql_lock();
            myedu_raise_irq(edu, FACT_IRQ);
            bql_unlock();
        }
    }

    return NULL;
}

static void pci_myedu_realize(PCIDevice *pdev, Error **errp)
{
    MyEduState *edu = MYEDU(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&edu->dma_timer, QEMU_CLOCK_VIRTUAL, myedu_dma_timer, edu);

    qemu_mutex_init(&edu->thr_mutex);
    qemu_cond_init(&edu->thr_cond);
    qemu_thread_create(&edu->thread, "myedu", myedu_fact_thread,
                       edu, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&edu->mmio, OBJECT(edu), &myedu_mmio_ops, edu,
                    "myedu-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &edu->mmio);
}

static void pci_myedu_uninit(PCIDevice *pdev)
{
    MyEduState *edu = MYEDU(pdev);

    qemu_mutex_lock(&edu->thr_mutex);
    edu->stopping = true;
    qemu_mutex_unlock(&edu->thr_mutex);
    qemu_cond_signal(&edu->thr_cond);
    qemu_thread_join(&edu->thread);

    qemu_cond_destroy(&edu->thr_cond);
    qemu_mutex_destroy(&edu->thr_mutex);

    timer_del(&edu->dma_timer);
    msi_uninit(pdev);
}

static void myedu_instance_init(Object *obj)
{
    MyEduState *edu = MYEDU(obj);

    edu->dma_mask = (1UL << 28) - 1;
    object_property_add_uint64_ptr(obj, "dma_mask",
                                   &edu->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void myedu_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_myedu_realize;
    k->exit = pci_myedu_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x11e9;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_myedu_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo edu_info = {
        .name          = TYPE_PCI_MYEDU_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(MyEduState),
        .instance_init = myedu_instance_init,
        .class_init    = myedu_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&edu_info);
}
type_init(pci_myedu_register_types)
