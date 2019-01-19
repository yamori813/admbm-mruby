
#include "cfe.h"
#define blockcopy memcpy
#include "cfe_irq.h"

#include "lib_try.h"
#include "lib_physio.h"

#include "cfe_cache.h"
#include "net_enet.h"

#include "pci_machdep.h"
#include "sbmips32.h"
#include "lib_hssubr.h"
#include "addrspace.h"

#include "sb_bp.h"
#include "sb_pci.h"
#include "sb_mac.h"

#include "adm5120.h"
#include "adm5120_eth.h"

extern void dcache_wback_inv(unsigned long start, unsigned long size);
extern void dcache_wback(unsigned long start, unsigned long size);
extern void dcache_inv(unsigned long start, unsigned long size);

#define dma_cache_wback_inv(start,size) dcache_wback_inv(start,size)
#define dma_cache_wback(start,size)     dcache_wback(start,size)
#define dma_cache_inv(start,size)       dcache_inv(start,size)

#define SYNC mips_wbflush()

/* MII extensions shared by several Broadcom PHYs */
#define R_INTERRUPT             0x1A

#define ETH_PORT_CPU    0x40
#define ETH_PORT_GIGA   0x20    /* GMII port */
#define ETH_PORT_4      0x10
#define ETH_PORT_3      0x08
#define ETH_PORT_2      0x04
#define ETH_PORT_1      0x02
#define ETH_PORT_0      0x01

#define BUFSIZE         1536

#define DESC_OWN_SC     0x80000000
#define DESC_RING_END   0x10000000

#define PHYS_PORTS      5
#define VLAN_PORTS      6

#define ETH_INT_MASK    0x01fdefff

#include "mii.h"

/*
   This is a driver for the Broadcom 4401 10/100 MAC with integrated
   PHY as well as for the 10/100 MAC cores used in OCP-based SOCs.

   This driver takes advantage of DMA coherence in systems that
   support it (e.g., SB1250).  For systems without coherent DMA (e.g,
   BCM47xx SOCs), packet buffer memory is explicitly flushed, and
   descriptors are referenced only in uncacheable modes.

   The ADM5120 does not have a big-endian mode for DMA.  On the
   SB1250, this driver therefore uses "preserve byte lanes" addresses
   for all DMA accesses that cross the ZBbus-PCI bridge.  Descriptors
   and packets headers as seen by the CPU must be byte-swapped for the
   DMA engine.  For hardware without such a mode, only little-endian
   operation is supported.
*/

#ifndef B44_DEBUG
#define B44_DEBUG 0
#endif

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "dev_sb_mac: system endian not set"
#endif

/* Set IPOLL to drive processing through the pseudo-interrupt
   dispatcher.  Set XPOLL to drive processing by an external polling
   agent.  Setting both is ok. */

#ifndef IPOLL
#define IPOLL 0
#endif
#ifndef XPOLL
#define XPOLL 1
#endif

#define CACHE_ALIGN       32
#define PAGE_ALIGN        4096
#define ALIGN(n,align)    (((n)+((align)-1)) & ~((align)-1))

#define MIN_ETHER_PACK  (ENET_MIN_PKT+ENET_CRC_SIZE)   /* size of min packet */
#define MAX_ETHER_PACK  (ENET_MAX_PKT+ENET_CRC_SIZE)   /* size of max packet */

/* Packet buffers.  For the ADM5120, an rx packet is preceded by
   status information written into the rx buffer.  The packet itself
   begins at a programmable offset (PKTBUF_RX_OFFSET), which must be
   at least 28.  The DMA engine allows arbitrary buffer and packet
   alignment, but aligning to a cache line boundary can reduce lines
   touched on the copies.  Also, header and packet must be 32-bit
   aligned for 47xx byte swapping on RAM access to work correctly.  */

#define PKTBUF_RX_OFFSET    CACHE_ALIGN
#define ETH_PKTBUF_LEN      ALIGN(PKTBUF_RX_OFFSET+MAX_ETHER_PACK, CACHE_ALIGN)
#define ETH_PKTPOOL_SIZE    32

/* Descriptor structures.  The descriptor ring must begin on a 4K
   boundary and cannot exceed 512 entries.  Note that descriptors are
   referenced by the DMA engine using match-bytes addresses. */

typedef struct rx_dscr {
    pci_addr_t rxd_bufptr1;
    pci_addr_t rxd_bufptr2;
    uint32_t   rxd_buf1len;
    uint32_t   rxd_flags;
} rx_dscr;
	
typedef struct tx_dscr {
    pci_addr_t txd_bufptr1;
    pci_addr_t txd_bufptr2;
    uint32_t   txd_buf1len;
    uint32_t   txd_flags;
} tx_dscr;


/* Driver data structures */

typedef enum {
    eth_state_uninit,
    eth_state_off,
    eth_state_on, 
    eth_state_broken
} eth_state_t;

typedef struct admsw_softc {
    uint32_t membase;
    uint8_t irq;		/* interrupt mapping (used if IPOLL) */
    pcitag_t tag;               /* tag for configuration registers */

    /* functions for mapping between memory and dma addresses */
    uint32_t (*phys_to_dma)(uint32_t);
    uint32_t (*dma_to_phys)(uint32_t);

    uint8_t hwaddr[ENET_ADDR_LEN];                 
    uint16_t device;            /* chip device code */
    uint8_t revision;		/* chip revision and step */

    eth_state_t state;          /* current state */
    uint32_t intmask;           /* interrupt mask */

    /* These fields are set before calling admsw_hwinit */
    int linkspeed;		/* encodings from cfe_ioctl */
    int loopback;

    /* Packet buffer */
    uint8_t *pktpool;

    /* The descriptor tables */
    uint8_t    *rxdscrmem;	/* receive descriptors */
    uint8_t    *txdscrmem;	/* transmit descriptors */
    uint8_t    *rxhpdscrmem;	/* receive descriptors */
    uint8_t    *txhpdscrmem;	/* transmit descriptors */

    /* These fields keep track of where we are in tx/rx processing */
    volatile rx_dscr *rxdscr_start;	/* beginning of ring */
    int rxdscrpos;
    uint8_t *rxbase;

    volatile rx_dscr *rxdscr_hp_start;	/* beginning of ring */
    int rxdscrhppos;
    uint8_t *rxhpbase;

    volatile tx_dscr *txdscr_start;	/* beginning of ring */
    int txdscrpos;
    uint8_t *txbase;

    volatile tx_dscr *txdscr_hp_start;	/* beginning of ring */

    cfe_devctx_t *devctx;

    int phy_addr;
    uint32_t phy_vendor;
    uint16_t phy_device;
    int phy_interrupt;			/* bcm5221-like MII interrupt reg */
    int slow_poll;

    /* Statistics */
    uint32_t inpkts;
    uint32_t outpkts;
    uint32_t interrupts;
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;

    uint32_t port_enable;
    uint32_t priv;
} admsw_softc;


admsw_softc *gsc;

/* Entry to and exit from critical sections (currently relative to
   interrupts only, not SMP) */

#if CFG_INTERRUPTS
#define CS_ENTER(sc) cfe_disable_irq(sc->irq)
#define CS_EXIT(sc)  cfe_enable_irq(sc->irq)
#else
#define CS_ENTER(sc) ((void)0)
#define CS_EXIT(sc)  ((void)0)
#endif


/* Chip parameterization */

#define GP_TIMER_HZ    62500000


/* Driver parameterization */

#define MAXRXDSCR	4
#define MAXTXDSCR	4
#define MINRXRING	4


/* Prototypes */

static void admsw_probe(cfe_driver_t *drv,
			   unsigned long probe_a, unsigned long probe_b, 
			   void *probe_ptr);


/* Address mapping macros.  Accesses in which the ADM5120 is the
   target are to registers and use match bits mode.  Accesses in which
   it is the initiator always assume little-endian responses and must
   use match bytes, per the macros below.  For big-endian hosts, the
   DMA status word must be byte-swapped. */

/* Note that PTR_TO_PHYS only works with 32-bit addresses, but then
   so does the ADM5120. */
#define PTR_TO_PHYS(x) (PHYSADDR((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)KERNADDR(a))

/* The DMA engine does not have a big-endian option for descriptors
   and data.  All its accesses through the host bridge use match bytes
   mode.  The CPU must construct descriptors and headers accordingly.
   PIO accesses to the configuration and host interface registers use
   match bits.  */

#if ENDIAN_BIG && defined(ADM5120)
/* XXX This needs cleanup/abstraction.  The 47xx SOCs efectively have
   an endian bit that is used to swap bytes in SDRAM accesses
   (only). */
static uint32_t phys_to_sb(uint32_t a) {return a + 0x10000000;}
static uint32_t sb_to_phys(uint32_t a) {return a - 0x10000000;}
#else
static uint32_t phys_to_sb(uint32_t a) {return a;}
static uint32_t sb_to_phys(uint32_t a) {return a;}
#endif

#undef PHYS_TO_PCI
#undef PCI_TO_PHYS

#define PCI_TO_PTR(sc,a)  (PHYS_TO_PTR((*(sc)->dma_to_phys)(a)))
#define PTR_TO_PCI(sc,x)  ((*(sc)->phys_to_dma)(PTR_TO_PHYS(x)))

#if ENDIAN_BIG
#define READCSR2(sc,csr) (phys_read16((sc)->membase + ((csr)^2)))
#define WRITECSR2(sc,csr,val) (phys_write16((sc)->membase + ((csr)^2), (val)))
#else
#define READCSR2(sc,csr) (phys_read16((sc)->membase + (csr)))
#define WRITECSR2(sc,csr,val) (phys_write16((sc)->membase + (csr), (val)))
#endif
#define READCSR(sc,csr) (phys_read32((sc)->membase + (csr)))
#define WRITECSR(sc,csr,val) (phys_write32((sc)->membase + (csr), (val)))

/* Byte swap utilities: host to/from little-endian */

#if ENDIAN_BIG
#define HTOL4(x) \
    ((((x) & 0x00FF) << 24) | \
     (((x) & 0xFF00) << 8)  | \
     (((x) >> 8) & 0xFF00)  | \
     (((x) >> 24) & 0x00FF))

static uint32_t
htol4(uint32_t x)
{
    uint32_t t;

    t = ((x & 0xFF00FF00) >> 8) | ((x & 0x00FF00FF) << 8);
    return (t >> 16) | ((t & 0xFFFF) << 16);
}
#else
#define HTOL4(x) (x)
#define htol4(x) (x)
#endif

#define ltoh4 htol4   /* self-inverse */


/* Utilities */

static const char *
admsw_devname(admsw_softc *sc)
{
//    return (sc->devctx != NULL ? cfe_device_name(sc->devctx) : "eth?");
    return ("eth0");
}

/*  Receive buffer processing. */

static void
admsw_procrxring(admsw_softc *sc)
{
    int i;
    volatile rx_dscr *rxd;
    int emp;
    int len;

    for (i = 0; i < MAXRXDSCR; ++i) {
//        CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
        rxd = &sc->rxdscr_hp_start[(sc->rxdscrhppos + i) % MAXRXDSCR];
//        dma_cache_inv(rxd, sizeof(rx_dscr));
	emp = (uint32_t)rxd->rxd_bufptr1 & DESC_OWN_SC;
        if (!emp) {
	    len = rxd->rxd_flags >> 16;
	    dma_cache_wback_inv(sc->rxhpbase + BUFSIZE * ((sc->rxdscrhppos + i) % MAXRXDSCR), len);
	    CS_ENTER(sc);
	    net_rcv(sc->rxhpbase + BUFSIZE * ((sc->rxdscrhppos + i) % MAXRXDSCR), len);
	    CS_EXIT(sc);
	    rxd->rxd_bufptr1 |= DESC_OWN_SC;
//	    CACHE_DMA_SYNC(rxd, sizeof(rx_dscr));
//            CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
            dma_cache_wback_inv(rxd, sizeof(rx_dscr));
        } else {
	    sc->rxdscrhppos = (sc->rxdscrhppos + i) % MAXRXDSCR;
	    break;
	}
    }
    for (i = 0; i < MAXRXDSCR; ++i) {
//        CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
        rxd = &sc->rxdscr_start[(sc->rxdscrpos + i) % MAXRXDSCR];
//        dma_cache_inv(rxd, sizeof(rx_dscr));
	emp = (uint32_t)rxd->rxd_bufptr1 & DESC_OWN_SC;
        if (!emp) {
	    len = rxd->rxd_flags >> 16;
	    dma_cache_wback_inv(sc->rxbase + BUFSIZE * ((sc->rxdscrpos + i) % MAXRXDSCR), len);
	    CS_ENTER(sc);
	    net_rcv(sc->rxbase + BUFSIZE * ((sc->rxdscrpos + i) % MAXRXDSCR), len);
	    CS_EXIT(sc);

	    rxd->rxd_bufptr1 |= DESC_OWN_SC;
//	    CACHE_DMA_SYNC(rxd, sizeof(rx_dscr));
//            CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
            dma_cache_wback_inv(rxd, sizeof(rx_dscr));
        } else {
	    sc->rxdscrpos = (sc->rxdscrpos + i) % MAXRXDSCR;
	    break;
	}
    }

    /* XXX Check for error stops. */
}

static void
admsw_proctxring(admsw_softc *sc)
{
}


static void
admsw_initrings(admsw_softc *sc)
{
    int i;
    volatile tx_dscr *txd;
    volatile rx_dscr *rxd;

    txd = sc->txdscr_start;
    sc->txbase = sc->pktpool;
    for (i = 0; i < MAXTXDSCR; ++i) {
	txd->txd_bufptr1 = (int)(sc->txbase + BUFSIZE * i) & 0x1ffffff;
	txd->txd_bufptr2 = 0;
        txd->txd_buf1len = BUFSIZE;
        txd->txd_flags = 0;
	CACHE_DMA_INVAL(txd, sizeof(tx_dscr));
	++txd;
	}
    (txd-1)->txd_bufptr1 |= HTOL4(DESC_RING_END);
    dma_cache_wback_inv(sc->txdscr_start, sizeof(tx_dscr) * MAXRXDSCR);

    rxd = sc->rxdscr_start;
    sc->rxbase = sc->pktpool + BUFSIZE * MAXTXDSCR;
    for (i = 0; i < MAXRXDSCR; ++i) {
	rxd->rxd_bufptr1 = (int)(sc->rxbase + BUFSIZE * i) & 0x1ffffff;
	rxd->rxd_bufptr1 |= DESC_OWN_SC;
	rxd->rxd_bufptr2 = 0;
        rxd->rxd_buf1len = BUFSIZE;
	rxd->rxd_flags = 0;
	CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
	++rxd;
	}
    (rxd-1)->rxd_bufptr1 |= HTOL4(DESC_RING_END);
    dma_cache_wback_inv(sc->rxdscr_start, sizeof(rx_dscr) * MAXRXDSCR);

    rxd = sc->rxdscr_hp_start;
    sc->rxhpbase = sc->pktpool + BUFSIZE * (MAXTXDSCR + MAXRXDSCR);
    for (i = 0; i < MAXRXDSCR; ++i) {
	rxd->rxd_bufptr1 = (int)(sc->rxhpbase + BUFSIZE * i) & 0x1ffffff;
	rxd->rxd_bufptr1 |= DESC_OWN_SC;
	rxd->rxd_bufptr2 = 0;
        rxd->rxd_buf1len = BUFSIZE;
	rxd->rxd_flags = 0;
//	CACHE_DMA_INVAL(rxd, sizeof(rx_dscr));
	++rxd;
	}
    (rxd-1)->rxd_bufptr1 |= HTOL4(DESC_RING_END);
    dma_cache_wback_inv(sc->rxdscr_hp_start, sizeof(rx_dscr) * MAXRXDSCR);

    /* not use */
    txd = sc->txdscr_hp_start;
    for (i = 0; i < MAXTXDSCR; ++i) {
	txd->txd_bufptr1 = 0;
	txd->txd_bufptr2 = 0;
        txd->txd_buf1len = BUFSIZE;
        txd->txd_flags = 0;
//	CACHE_DMA_INVAL(txd, sizeof(tx_dscr));
	++txd;
	}
    (txd-1)->txd_bufptr1 |= HTOL4(DESC_RING_END);
    dma_cache_wback_inv(sc->txdscr_hp_start, sizeof(tx_dscr) * MAXRXDSCR);

    mips_wbflush();


//    mips_wbflush();
    dma_cache_wback_inv(sc->pktpool,
	BUFSIZE * (MAXTXDSCR + MAXRXDSCR * 2));

    /* Precharge the receive ring */
}


/* Allocate an integral number of cache lines suitable for DMA access. */
static uint8_t *
dma_alloc(size_t size, unsigned int align)
{
    uint8_t *base;
    size_t len = ALIGN(size, CACHE_ALIGN);

    base = KMALLOC(len, ALIGN(align, CACHE_ALIGN));
    if (base != NULL)
	CACHE_DMA_INVAL(base, len);
    return base;
}

static int
admsw_init(admsw_softc *sc)
{
    /* Allocate descriptor rings */
    sc->rxdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXRXDSCR*sizeof(rx_dscr), 32));
    sc->txdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXTXDSCR*sizeof(tx_dscr), 32));
    sc->rxhpdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXRXDSCR*sizeof(rx_dscr), 32));
    sc->txhpdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXTXDSCR*sizeof(tx_dscr), 32));

    /* Allocate buffer pool (4K aligned to fix apparent DMA alignment bug) */
    sc->pktpool = dma_alloc((MAXTXDSCR + MAXRXDSCR * 2) * BUFSIZE,
	32);
    if (sc->pktpool == NULL) {
	xprintf("%s: No buffer memory available.\n", admsw_devname(sc));
	return -1;
	}

    /* Fill in pointers to the rings */
    sc->rxdscr_start = (volatile rx_dscr *) (sc->rxdscrmem);
    sc->rxdscrpos = 0;

    sc->txdscr_start = (volatile tx_dscr *) (sc->txdscrmem);
    sc->txdscrpos = 0;

    sc->rxdscr_hp_start = (volatile rx_dscr *) (sc->rxhpdscrmem);
    sc->rxdscrhppos = 0;

    sc->txdscr_hp_start = (volatile tx_dscr *) (sc->txhpdscrmem);

    admsw_initrings(sc);

    WRITECSR(sc, SCR_SEND_LBADDR, (int)sc->txdscr_start & 0x1ffffff);
    WRITECSR(sc, SCR_RECEIVE_LBADDR, (int)sc->rxdscr_start & 0x1ffffff);
    WRITECSR(sc, SCR_SEND_HBADDR, (int)sc->txdscr_hp_start & 0x1ffffff);
    WRITECSR(sc, SCR_RECEIVE_HBADDR, (int)sc->rxdscr_hp_start & 0x1ffffff);

    return 0;
}


/* CRC */


/* EEPROM access */


/* MII access */

static void
mii_enable(admsw_softc *sc)
{
    uint32_t devctl, enetctl;

    devctl = READCSR(sc, R_DEV_CONTROL);
    if ((devctl & M_DVCTL_IP) != 0) {
	WRITECSR(sc, R_MII_STATUS_CONTROL, M_MIICTL_PR | V_MIICTL_MD(0xD));
	devctl = READCSR(sc, R_DEV_CONTROL);
	if ((devctl & M_DVCTL_ER) != 0) {
	    devctl &= ~M_DVCTL_ER;
	    WRITECSR(sc, R_DEV_CONTROL, devctl);
	    cfe_usleep(100);
	    }
	}
    else {
	WRITECSR(sc, R_MII_STATUS_CONTROL, M_MIICTL_PR | V_MIICTL_MD(0x9));
	enetctl = READCSR(sc, R_ENET_CONTROL);
	enetctl |= M_ECTL_EP;
	WRITECSR(sc, R_ENET_CONTROL, enetctl);
	}
}

static uint16_t
mii_read(admsw_softc *sc, int reg)
{
    uint32_t cmd, status;
    uint32_t data;
    int timeout;

    WRITECSR(sc, R_ENET_INT_STATUS, M_EINT_MI);
    (void)READCSR(sc, R_ENET_INT_STATUS);

    cmd = (V_MIIDATA_OP(K_MII_OP_READ) | V_MIIDATA_TA(K_TA_VALID) |
           V_MIIDATA_RA(reg) | V_MIIDATA_PM(sc->phy_addr));
    WRITECSR(sc, R_MII_DATA, cmd | V_MIIDATA_SB(K_MII_START));

    for (timeout = 1000; timeout > 0; timeout -= 100) {
	status = READCSR(sc, R_ENET_INT_STATUS);
	if ((status & M_EINT_MI) != 0)
	    break;
	cfe_usleep(100);
	}

    if (timeout <= 0)
	return 0xFFFF;

    data = G_MIIDATA_D(READCSR(sc, R_MII_DATA));
    return data;
}

static void
mii_write(admsw_softc *sc, int reg, uint16_t value)
{
    uint32_t cmd, status;
    int timeout;

    WRITECSR(sc, R_ENET_INT_STATUS, M_EINT_MI);
    (void)READCSR(sc, R_ENET_INT_STATUS);

    cmd = (V_MIIDATA_OP(K_MII_OP_WRITE) | V_MIIDATA_TA(0x2) |
           V_MIIDATA_RA(reg) | V_MIIDATA_PM(sc->phy_addr) |
	   V_MIIDATA_D(value));
    WRITECSR(sc, R_MII_DATA, cmd | V_MIIDATA_SB(K_MII_START));

    for (timeout = 1000; timeout > 0; timeout -= 100) {
	status = READCSR(sc, R_ENET_INT_STATUS);
	if ((status & M_EINT_MI) != 0)
	    break;
	cfe_usleep(100);
	}
}

/* For an integrated PHY (admsw), unimplemented PHY addresses return
   id's of 0.  For (some?) external PHYs, unimplmented addresses
   appear to return 0x1FFF or 0x3FFF for id1 but reliably (?) return
   0xFFFF for id2.  */
static int
mii_probe(admsw_softc *sc)
{
    int i;
    uint16_t id1, id2;
    int prev = sc->phy_addr;

    for (i = 0; i < 32; i++) {
        sc->phy_addr = i;
        id1 = mii_read(sc, R_PHYIDR1);
	id2 = mii_read(sc, R_PHYIDR2);
	if (id2 != 0x0000 && id2 != 0xFFFF) {
	    sc->phy_vendor = ((uint32_t)id1 << 6) | ((id2 >> 10) & 0x3F);
	    sc->phy_device = (id2 >> 4) & 0x3F;
	    xprintf("phy %d, vendor %06x part %02x\n",
		    i, sc->phy_vendor, sc->phy_device);
	    
#if 0
	    return 0;
#endif
	    }
	}
#if 0
    xprintf("mii_probe: No PHY found\n");
#else
    xprintf("mii_probe: Using PHY %d\n", prev);
#endif
    sc->phy_addr = prev;   /* Expected addr (if any) */
    return -1;
}

#if B44_DEBUG
static void
mii_dump(admsw_softc *sc, const char *label)
{
    int i;
    uint16_t  r;
    uint32_t  idr, part;

    xprintf("%s, MII:\n", label);
    idr = part = 0;

    /* Common registers */
    for (i = 0x0; i <= 0x8; ++i) {
	r = mii_read(sc, i);
	xprintf(" REG%02X: %04X", i, r);
	if (i % 4 == 3) xprintf("\n");
	if (i == MII_PHYIDR1) {
	    idr |= r << 6;
	    }
	else if (i == MII_PHYIDR2) {
	    idr |= (r >> 10) & 0x3F;
	    part = (r >> 4) & 0x3F;
	    }
	}
    xprintf("\nIDR %06x, PART %02x\n", idr, part);
    
    /* Broadcom extensions */
    for (i = 0x10; i <= 0x14; ++i) {
	r = mii_read(sc, i);
	xprintf(" REG%02X: %04X", i, r);
	if (i % 4 == 3) xprintf("\n");
	}
    xprintf("\n");

    /* Broadcom extensions (52xx family) */
    for (i = 0x18; i <= 0x1F; i++) {
	r = mii_read(sc, i);
	xprintf(" REG%02X: %04X", i, r);
	if (i % 4 == 3) xprintf("\n");
	}
    xprintf("\n");
}
#else
#define mii_dump(sc,label)
#endif

static void
mii_set_speed(admsw_softc *sc, int speed)
{
    /* NYI */
}

static uint16_t
mii_interrupt(admsw_softc *sc)
{
    /* The read also clears any interrupt bits. */
    return mii_read(sc, R_INTERRUPT);
}

#define ROBOSW_ACCESS_CONTROL_REG	0x10
#define ROBOSW_RW_CONTROL_REG		0x11
#define ROBOSW_DATA_REG_BASE		0x18

#define ACCESS_CONTROL_RW		0x0001
#define RW_CONTROL_WRITE		0x0001
#define RW_CONTROL_READ			0x0002

static void
mii_autonegotiate(admsw_softc *sc)
{
    uint16_t  control, status, remote;
    unsigned int  timeout;
    int linkspeed;

    linkspeed = ETHER_SPEED_UNKNOWN;

    xprintf("%s: Link speed: ", admsw_devname(sc));

    if (sc->phy_addr == 0x1E) {
	/* XXX for SOC parts, this address may indicate a Roboswitch. */
	xprintf("100BaseT FDX (switch)\n");
	linkspeed = ETHER_SPEED_100FDX;	 
    }
    else {
	/* Read twice to clear latching bits */
	status = mii_read(sc, MII_BMSR);
	status = mii_read(sc, MII_BMSR);

	if ((status & (BMSR_AUTONEG | BMSR_LINKSTAT)) ==
	    (BMSR_AUTONEG | BMSR_LINKSTAT))
	    control = mii_read(sc, MII_BMCR);
	else {
	    for (timeout = 4*CFE_HZ; timeout > 0; timeout -= CFE_HZ/2) {
		status = mii_read(sc, MII_BMSR);
		if ((status & BMSR_ANCOMPLETE) != 0)
		    break;
		cfe_sleep(CFE_HZ/2);
		}
	    }

	remote = mii_read(sc, MII_ANLPAR);
	if ((status & BMSR_ANCOMPLETE) != 0) {
	    /* A link partner was negotiated... */

	    if ((remote & ANLPAR_TXFD) != 0) {
		xprintf("100BaseT FDX\n");
		linkspeed = ETHER_SPEED_100FDX;	 
		}
	    else if ((remote & ANLPAR_TXHD) != 0) {
		xprintf("100BaseT HDX\n");
		linkspeed = ETHER_SPEED_100HDX;	 
		}
	    else if ((remote & ANLPAR_10FD) != 0) {
		xprintf("10BaseT FDX\n");
		linkspeed = ETHER_SPEED_10FDX;	 
		}
	    else if ((remote & ANLPAR_10HD) != 0) {
		xprintf("10BaseT HDX\n");
		linkspeed = ETHER_SPEED_10HDX;	 
		}
	    }
	else {
	    /* no link partner convergence */
	    xprintf("Unknown\n");
	    linkspeed = ETHER_SPEED_UNKNOWN;
	    }
	sc->linkspeed = linkspeed;

	/* clear latching bits */
	status = mii_read(sc, MII_BMSR);
	}

    mii_dump(sc, "final PHY");
}


static int
admsw_reset(admsw_softc *sc)
{
    return -1;
}

static int
admsw_set_hw_addr(admsw_softc *sc, uint8_t addr[])
{
    uint32_t enet_upper, enet_lower;
    int timeout;

    enet_upper = (addr[0] << 8) | addr[1];
    enet_lower = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
    
    WRITECSR(sc, R_CAM_DATA_H, M_CAM_VB | V_CAM_CD_H(enet_upper));
    WRITECSR(sc, R_CAM_DATA_L, V_CAM_CD_L(enet_lower));

    WRITECSR(sc, R_CAM_CONTROL, V_CAMCTL_IX(0) | M_CAMCTL_CW);
    for (timeout = CFE_HZ; timeout > 0; timeout -= CFE_HZ/10) {
	if ((READCSR(sc, R_CAM_CONTROL) & M_CAMCTL_CB) != 0)
	    break;
	cfe_sleep(1);
	}
    if (timeout <= 0)
        return -1;

    return 0;
}


static void
admsw_set_linkspeed(admsw_softc *sc)
{
    uint32_t ctrl;

    ctrl = READCSR(sc, R_XMT_CONTROL1);
    switch (sc->linkspeed) {
	case ETHER_SPEED_100FDX:
	case ETHER_SPEED_10FDX:
	    ctrl |= (M_TCTL_FD | M_TCTL_SB);
	    break;
	default:
	    ctrl &= ~(M_TCTL_FD | M_TCTL_SB);
	    break;
	}
    WRITECSR(sc, R_XMT_CONTROL1, ctrl);
}

static void
admsw_hwinit(admsw_softc *sc)
{
    WRITECSR(sc, SCR_CPUP_CONF, READCSR(sc, SCR_CPUP_CONF) & ~0x01);

    WRITECSR(sc, SCR_MAC_WT1,
	(sc->hwaddr[5] << 24) | (sc->hwaddr[4] << 16) |
	(sc->hwaddr[3] << 8) | sc->hwaddr[2]);
    WRITECSR(sc, SCR_MAC_WT0, 
	(sc->hwaddr[1] << 24) | (sc->hwaddr[0] << 16) |
	((unsigned)sc->priv << 3) | 0x2041);

    SYNC;

    while ((READCSR(sc, SCR_MAC_WT0) & 0x2) == 0)      /* wtMAC_done */
	;

    WRITECSR(sc, SCR_INT_MSK, ETH_INT_MASK);
    WRITECSR(sc, SCR_INT_ST, READCSR(sc, SCR_INT_ST));

    SYNC;
}

static void
admsw_setspeed(admsw_softc *sc, int speed)
{
    /* XXX Not yet implemented - autonegotiation only. */
    (void)mii_set_speed;
}

static void
admsw_setloopback(admsw_softc *sc, int mode)
{
    /* XXX Not yet implemented. */
}


static void
admsw_isr(void *arg)
{
    admsw_softc *sc = (admsw_softc *)arg;
    uint32_t status;

#if IPOLL
    sc->interrupts++;
#endif
    admsw_procrxring(sc);
    WRITECSR(sc, SCR_INT_ST, READCSR(sc, SCR_INT_ST));
#if 0

    for (;;) {

	/* Read and clear the interrupt status. */
	status = READCSR(sc, R_INT_STATUS);
	status &= sc->intmask;
	if (status == 0)
	    break;

	WRITECSR(sc, R_INT_STATUS, status);  /* write-to-clear */

	/* XXX Handle SERR, etc. */

	if (status & M_INT_RI) {
#if IPOLL
	    sc->rx_interrupts++;
#endif
	    admsw_procrxring(sc);
	    }

	if (status & M_INT_XI) {
#if IPOLL
	    sc->tx_interrupts++;
#endif
	    admsw_proctxring(sc);
	    }

	if (status & M_INT_TO) {
	    sc->slow_poll = 1;
	    }

	if (status & (M_INT_XU | M_INT_RO)) {
	    if (status & M_INT_XU) {
		xprintf("ADM5120: tx underrun, %08x\n", status);
		/* XXX Try to restart */
		}
	    if (status & M_INT_RO) {
		xprintf("ADM5120: rx overrun, %08x\n", status);
		/* XXX Try to restart */
		}
	    }
	}
#endif
}


static void
admsw_start(admsw_softc *sc)
{
    uint32_t status;

    admsw_hwinit(sc);
    sc->irq = 0;
#if IPOLL
    cfe_request_irq(sc->irq, admsw_isr, sc, CFE_IRQ_FLAGS_SHARED, 0);
    adm_irq_enable(INT_LVL_SWITCH);
    WRITECSR(sc, SCR_INT_MSK, 0x1fdeff0);
#endif

    sc->state = eth_state_on;
}

static void
admsw_stop(admsw_softc *sc)
{
    uint32_t ctl, status;
    int i;

    /* Cancel the timer */
    if (sc->phy_interrupt) {
	WRITECSR(sc, R_GP_TIMER, 0);
	(void)READCSR(sc, R_GP_TIMER);
	}
    sc->slow_poll = 0;

    /* Make sure that no further interrupts will be processed. */
    sc->intmask = 0;
    WRITECSR(sc, R_INT_MASK, 0);
    (void)READCSR(sc, R_INT_MASK);  /* push */
    status = READCSR(sc, R_INT_STATUS);
    WRITECSR(sc, R_INT_STATUS, status);   /* write-to-clear */

    /* Shut down MAC */
    WRITECSR(sc, R_ENET_CONTROL, M_ECTL_ED);
    for (i = 1000; i > 0; i--) {
	ctl = READCSR(sc, R_ENET_CONTROL);
	if ((ctl & M_ECTL_ED) == 0)
	    break;
	cfe_usleep(100);
	}
    if (i == 0)
	xprintf("%s: cannot clear MAC\n", admsw_devname(sc));

    /* Shut down DMA engines */
    WRITECSR(sc, R_XMT_CONTROL, 0);
    for (i = 1000; i > 0; i--) {
	status = READCSR(sc, R_XMT_STATUS);
	if (G_XSTAT_XS(status) == K_XS_DISABLED)
	    break;
	cfe_usleep(100);
	}
    if (i == 0)
	xprintf("%s: cannot clear tx DMA\n", admsw_devname(sc));

    WRITECSR(sc, R_RCV_CONTROL, 0);
    for (i = 1000; i > 0; i--) {
	status = READCSR(sc, R_RCV_STATUS);
	if (G_RSTAT_RS(status) == K_RS_DISABLED)
	    break;
	cfe_usleep(100);
	}
    if (i == 0)
	xprintf("%s: cannot clear rx DMA\n", admsw_devname(sc));

    status = READCSR(sc, R_INT_STATUS);
    WRITECSR(sc, R_INT_STATUS, status);
#if IPOLL
    cfe_free_irq(sc->irq, 0);
#endif

    /* Leave the mii inteface enabled */
    mii_enable(sc);
}


/* Declarations for CFE Device Driver Interface routines */

static int admsw_ether_open(cfe_devctx_t *ctx);
static int admsw_ether_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int admsw_ether_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int admsw_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int admsw_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int admsw_ether_close(cfe_devctx_t *ctx);
static void admsw_ether_poll(cfe_devctx_t *ctx, int64_t ticks);
static void admsw_ether_reset(void *softc);


/* CFE Device Driver dispatch structure */


/* CFE Device Driver probe functions for SOC core. */

const cfe_driver_t sb_mac = {
    "ADM5120 Ethernet",
    "eth",
    CFE_DEV_NETWORK,
    0,
    admsw_probe
};

static uint8_t port_map[VLAN_PORTS] = {
        ETH_PORT_CPU | ETH_PORT_4 | ETH_PORT_3 | ETH_PORT_2 | ETH_PORT_1 | ETH_PORT_0, 0, 0, 0, 0, 0
};

void
switch_init(admsw_softc *sc)
{
int i;

    WRITECSR(sc, SCR_CPUP_CONF, 0x01);
    WRITECSR(sc, SCR_PORT_CONF0, 0x3f);
    SYNC;
    cfe_usleep(100000);

    WRITECSR(sc, SCR_VLAN_GI,
	port_map[0] | (port_map[1] << 8) |
	(port_map[2] << 16) | (port_map[3] << 24));
    WRITECSR(sc, SCR_VLAN_GII,
	port_map[4] | (port_map[5] << 8));

    SYNC;

    sc->port_enable = 0;
    for (i = 0; i < VLAN_PORTS; i++)
	sc->port_enable |= port_map[i];

    /* CPU port to be disabled, CRC padding, disable unknown packets */
    WRITECSR(sc, SCR_CPUP_CONF, 0x01 |
	0x02 |  /* CRC padding */
	((sc->port_enable & 0x3f) << 9)); /* disable unknown packets */
    WRITECSR(sc, SCR_PORT_CONF0,
	(~sc->port_enable & 0x3f) |
	((sc->port_enable & 0x3f) << 16) |
	((sc->port_enable & 0x3f) << 8));

    WRITECSR(sc, SCR_PORT_CONF1, (sc->port_enable & 0x3f) << 20);

    if (sc->port_enable & ETH_PORT_GIGA) {
	WRITECSR(sc, SCR_PORT_CONF2,
	    0x01 | /* AN enable */
	    0x02 |/* 100M */   
	    0x08 | /* full duplex */
	    0x30 /* flow control */);
    } else {
	WRITECSR(sc, SCR_PORT_CONF2,
	    0x02 |/* 100M */
	    0x08 | /* full duplex */
	    0x30 | /* flow control */
	    0x100 /* disable check TXC */ );
    }

    SYNC;

    cfe_usleep(100000);

    WRITECSR(sc, SCR_PHY_CNTL2,
	(sc->port_enable & 0x1f) |          /* auto negotiation */
	((sc->port_enable & 0x1f) << 5) |   /* speed control */
	((sc->port_enable & 0x1f) << 10) |  /* duplex control */
	((sc->port_enable & 0x1f) << 15) |  /* FC rec */
	((sc->port_enable & 0x1f) << 20) |  /* disable reset */
	((sc->port_enable & 0x1f) << 25) |
	0x80000000);                    /* XXX undocumented */

    WRITECSR(sc, SCR_PHY_CNTL3, 0x0001410b);

    WRITECSR(sc, SCR_INT_MSK, ETH_INT_MASK);
    WRITECSR(sc, SCR_INT_ST, READCSR(sc, SCR_INT_ST));

    SYNC;
}



static int
admsw_attach(cfe_driver_t *drv,
		const char *mac_addr, int phy_addr)
{
    admsw_softc *sc;
    phys_addr_t pa;
    uint32_t flag, id;
    char descr[100];

    /* XXX The map of enumeration space should be discovered by scanning. */
    static const phys_addr_t enet_base[1] = {SWCTRL_BASE};

    /* Use memory space for the CSRs */
    pa = enet_base[0];

    sc = (admsw_softc *) KMALLOC(sizeof(admsw_softc), CACHE_ALIGN);
    if (sc == NULL) {
	xprintf("ADM5120 MAC: No memory to complete probe\n");
	return 0;
	}
    memset(sc, 0, sizeof(*sc));

    sc->priv = 0;

    sc->membase = (uint32_t)pa;

    sc->phys_to_dma = phys_to_sb;
    sc->dma_to_phys = sb_to_phys;

    sc->devctx = NULL;

    sc->linkspeed = ETHER_SPEED_AUTO;    /* select autonegotiation */
    sc->loopback = ETHER_LOOPBACK_OFF;

    switch_init(sc);

//    mii_enable(sc);

    admsw_init(sc);

    if (mac_addr) {
	enet_parse_hwaddr(mac_addr, sc->hwaddr);
	}
    else {
	sc->hwaddr[0] = 0x02; sc->hwaddr[1] = 0x00; sc->hwaddr[2] = 0x00;
	sc->hwaddr[3] = 0x47; sc->hwaddr[4] = 0x10; sc->hwaddr[5] = 0x10;
	}
    sc->phy_addr = phy_addr;

    sc->state = eth_state_uninit;

    xsprintf(descr, "%s at 0x%X (%a)",
	     drv->drv_description, sc->membase, sc->hwaddr);

//    cfe_attach(drv, sc, NULL, descr);
    gsc = sc;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    admsw_start(sc);

    return 1;
}

/*
 *  Probe arguments:
 *         probe_a - index of the MAC to probe
 *         probe_b - PHY address for MII
 *         probe_ptr - string pointer to hardware address for this
 *                     MAC, in the form xx:xx:xx:xx:xx:xx.
 */


static void
admsw_probe(cfe_driver_t *drv,
		    unsigned long probe_a, unsigned long probe_b, 
		    void *probe_ptr)
{
    int index = probe_a;
    int phy_addr = probe_b;
    const char *mac_addr = probe_ptr;

    admsw_attach(drv, mac_addr, phy_addr);
}


/* The functions below are called via the dispatch vector for the 4401. */

static int
admsw_ether_open(cfe_devctx_t *ctx)
{
    admsw_softc *sc = ctx->dev_softc;

    if (sc->state == eth_state_on)
	admsw_stop(sc);

    sc->devctx = ctx;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    admsw_start(sc);

#if XPOLL
    admsw_isr(sc);
#endif

    return 0;
}

static int
admsw_ether_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    admsw_softc *sc = ctx->dev_softc;
    int   mode;
    int   speed;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_ETHER_GETHWADDR:
	    hs_memcpy_to_hs(buffer->buf_ptr, sc->hwaddr, sizeof(sc->hwaddr));
	    return 0;

	case IOCTL_ETHER_SETHWADDR:
	    return -1;    /* not supported */

	case IOCTL_ETHER_GETSPEED:
	    speed = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr, &speed, sizeof(speed));
	    return 0;

	case IOCTL_ETHER_SETSPEED:
	    hs_memcpy_from_hs(&speed,buffer->buf_ptr, sizeof(speed));
	    admsw_setspeed(sc, speed);
	    return -1;    /* not supported yet */

	case IOCTL_ETHER_GETLINK:
	    speed = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr, &speed, sizeof(speed));
	    return 0;

	case IOCTL_ETHER_GETLOOPBACK:
	    speed = sc->loopback;
	    hs_memcpy_to_hs(buffer->buf_ptr, &speed, sizeof(speed));
	    return 0;

	case IOCTL_ETHER_SETLOOPBACK:
	    hs_memcpy_from_hs(&mode,buffer->buf_ptr, sizeof(mode));
	    sc->loopback = ETHER_LOOPBACK_OFF;  /* default */
	    if (mode == ETHER_LOOPBACK_INT || mode == ETHER_LOOPBACK_EXT) {
		admsw_setloopback(sc, mode);
		}
	    return -1;    /* not supported yet */

	default:
	    return -1;
	}
}

static int
admsw_ether_close(cfe_devctx_t *ctx)
{
    admsw_softc *sc = ctx->dev_softc;

    sc->state = eth_state_off;
    admsw_stop(sc);

    xprintf("%s: %d sent, %d received, %d interrupts\n",
	    admsw_devname(sc), sc->outpkts, sc->inpkts, sc->interrupts);
    if (IPOLL) {
	xprintf("  %d tx interrupts, %d rx interrupts\n",
		sc->tx_interrupts, sc->rx_interrupts);
	}

    sc->devctx = NULL;
#if 1 /* XXX Redo partitioning among hwinit, start and stop */
    sc->state = eth_state_uninit;
#endif
    return 0;
}

static void
admsw_ether_reset(void *softc)
{
    admsw_softc *sc = (admsw_softc *)softc;

    /* Turn off the Ethernet interface. */
    if (sc->state == eth_state_on)
	admsw_stop(sc);
    admsw_reset(sc);

    sc->state = eth_state_uninit;
}

int
cfe_output(int buf_length, char *buf_ptr)
{
    admsw_softc *sc = gsc;
    char *buf;

#if XPOLL
    admsw_isr(sc);
#endif
    buf = sc->txbase + BUFSIZE * sc->txdscrpos;
    memcpy(buf, buf_ptr, buf_length);
    sc->txdscr_start[sc->txdscrpos].txd_buf1len = buf_length;
    if (buf_length < 60)
	buf_length = 60;
    sc->txdscr_start[sc->txdscrpos].txd_flags = (buf_length << 16) |
	(1 << sc->priv);
    dma_cache_wback_inv((unsigned long)&sc->txdscr_start[sc->txdscrpos],
	sizeof(tx_dscr));
    dma_cache_wback_inv(buf, buf_ptr);

    sc->txdscr_start[sc->txdscrpos].txd_bufptr1 |= DESC_OWN_SC;
    WRITECSR(sc, SCR_SEND_TRIG, 1);
    sc->txdscrpos = (sc->txdscrpos + 1) % MAXTXDSCR;
    SYNC;

#if XPOLL
    admsw_isr(sc);
#endif

    return 0;
}

void cfe_ether_init(char *macaddr)
{
cfe_driver_t dev;

    admsw_probe(&dev, 0, 0, macaddr);
}
