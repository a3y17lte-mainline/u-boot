// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos Generic Board Source (for mobile devices)
 *
 * Copyright (c) 2025 Kaustabh Chakraborty <kauschluss@disroot.org>
 */

#include <asm/armv8/mmu.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <blk.h>
#include <bootflow.h>
#include <ctype.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <i2c.h>
#include <efi.h>
#include <efi_loader.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <linux/sizes.h>
#include <lmb.h>
#include <part.h>
#include <stdbool.h>
#include <string.h>

DECLARE_GLOBAL_DATA_PTR;

#define lmb_alloc(size, addr) \
	lmb_alloc_mem(LMB_MEM_ALLOC_ANY, SZ_2M, addr, size, LMB_NONE)

struct efi_fw_image fw_images[] = {
	{
		.fw_name = u"UBOOT_BOOT_PARTITION",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = NULL,
	.images = fw_images,
	.num_images = ARRAY_SIZE(fw_images),
};

/*
 * The memory mapping includes all DRAM banks, along with the
 * peripheral block, and a sentinel at the end. This is filled in
 * dynamically.
 */
static struct mm_region exynos_mem_map[CONFIG_NR_DRAM_BANKS + 2] = {
	{
		/* Peripheral MMIO block */
		.virt = 0x10000000UL,
		.phys = 0x10000000UL,
		.size = 0x10000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE | PTE_BLOCK_PXN | PTE_BLOCK_UXN,
	},
};

struct mm_region *mem_map = exynos_mem_map;

static const char *exynos_prev_bl_get_bootargs(void)
{
	void *prev_bl_fdt_base = (void *)get_prev_bl_fdt_addr();
	int chosen_node_offset, ret;
	const struct fdt_property *bootargs_prop;

	ret = fdt_check_header(prev_bl_fdt_base);
	if (ret < 0) {
		log_err("%s: FDT is invalid (FDT_ERR %d)\n", __func__, ret);
		return NULL;
	}

	ret = fdt_path_offset(prev_bl_fdt_base, "/chosen");
	chosen_node_offset = ret;
	if (ret < 0) {
		log_err("%s: /chosen node not found (FDT_ERR %d)\n", __func__,
			ret);
		return NULL;
	}

	bootargs_prop = fdt_get_property(prev_bl_fdt_base, chosen_node_offset,
					 "bootargs", &ret);
	if (!bootargs_prop) {
		log_err("%s: /chosen/bootargs property not found (FDT_ERR %d)\n",
			__func__, ret);
		return NULL;
	}

	return bootargs_prop->data;
}

static void exynos_parse_dram_banks(const void *fdt_base)
{
	u64 mem_addr, mem_size = 0;
	u32 na, ns, i;
	int index = 1;
	int offset;

	if (fdt_check_header(fdt_base) < 0)
		return;

	/* #address-cells and #size-cells as defined in the fdt root. */
	na = fdt_address_cells(fdt_base, 0);
	ns = fdt_size_cells(fdt_base, 0);

	fdt_for_each_subnode(offset, fdt_base, 0) {
		if (strncmp(fdt_get_name(fdt_base, offset, NULL), "memory", 6))
			continue;

		for (i = 0; ; i++) {
			if (index > CONFIG_NR_DRAM_BANKS)
				break;

			mem_addr = fdtdec_get_addr_size_fixed(fdt_base, offset,
							      "reg", i, na, ns,
							      &mem_size, false);
			if (mem_addr == FDT_ADDR_T_NONE)
				break;

			if (!mem_size)
				continue;

			mem_map[index].phys = mem_addr;
			mem_map[index].virt = mem_addr;
			mem_map[index].size = mem_size;
			mem_map[index].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
					       PTE_BLOCK_INNER_SHARE;
			index++;
		}
	}
}

static void exynos_env_setup(void)
{
	const char *bootargs = exynos_prev_bl_get_bootargs();
	const char *dev_compatible, *soc_compatible;
	char *ptr;
	char buf[128];
	int nr_compatibles;
	int offset;
	int ret;

	if (bootargs) {
		/* Read the cmdline property which stores the serial number. */
		ret = cmdline_get_arg(bootargs, "androidboot.serialno", &offset);
		if (ret > 0) {
			strlcpy(buf, bootargs + offset, ret);
			env_set("serial#", buf);
		}
	}

	nr_compatibles = ofnode_read_string_count(ofnode_root(), "compatible");
	if (nr_compatibles < 2) {
		log_warning("%s: expected 2 or more compatible strings\n",
			    __func__);
		return;
	}

	ret = ofnode_read_string_index(ofnode_root(), "compatible",
				       nr_compatibles - 1, &soc_compatible);
	if (ret) {
		log_warning("%s: failed to read SoC compatible\n",
			    __func__);
		return;
	}

	ret = ofnode_read_string_index(ofnode_root(), "compatible", 0,
				       &dev_compatible);
	if (ret) {
		log_warning("%s: failed to read device compatible\n",
			    __func__);
		return;
	}

	/* <manufacturer>,<soc> => platform = <soc> */
	ptr = strchr(soc_compatible, ',');
	if (ptr)
		soc_compatible = ptr + 1;
	env_set("platform", soc_compatible);

	/* <manufacturer>,<codename> => board = <manufacturer>-<codename> */
	strlcpy(buf, dev_compatible, sizeof(buf) - 1);
	ptr = strchr(buf, ',');
	if (ptr)
		*ptr = '-';
	env_set("board", buf);

	/*
	 * NOTE: Board name usually goes as <manufacturer>-<codename>, but
	 * upstream device trees for Exynos SoCs are <soc>-<codename>.
	 * Extraction of <codename> from the board name is required.
	 */
	ptr = strchr(dev_compatible, ',');
	if (ptr)
		dev_compatible = ptr + 1;

	/* EFI booting requires the path to correct DTB, specify it here. */
	snprintf(buf, sizeof(buf), "exynos/%s-%s.dtb", soc_compatible,
		 dev_compatible);
	env_set("fdtfile", buf);
}

static int exynos_blk_env_setup(void)
{
	const char *blk_ifname;
	int blk_dev = 0;
	struct blk_desc *blk_desc;
	struct disk_partition info = {0};
	unsigned long largest_part_start = 0, largest_part_size = 0;
	static char dfu_string[32];
	int i;

	blk_ifname = "mmc";
	blk_desc = blk_get_dev(blk_ifname, blk_dev);
	if (!blk_desc) {
		log_err("%s: required mmc device not available\n", __func__);
		return -ENODEV;
	}

	for (i = 1; i < CONFIG_EFI_PARTITION_ENTRIES_NUMBERS; i++) {
		if (part_get_info(blk_desc, i, &info))
			continue;

		if (!update_info.dfu_string &&
		    !strncasecmp(info.name, "boot", strlen("boot"))) {
			snprintf(dfu_string, sizeof(dfu_string),
				 "mmc %d=u-boot.bin part %d %d", blk_dev,
				 blk_dev, i);
			update_info.dfu_string = dfu_string;
		}

		if (info.start > largest_part_size) {
			largest_part_start = info.start;
			largest_part_size = info.size;
		}
	}

	if (largest_part_size) {
		env_set("blkmap_blk_ifname", blk_ifname);
		env_set_ulong("blkmap_blk_dev", blk_dev);
		env_set_ulong("blkmap_blk_nr", largest_part_start);
		env_set_hex("blkmap_size_r", largest_part_size);
	} else {
		log_warning("%s: no qualified partition for blkmap, skipping\n",
			    __func__);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_FASTBOOT_FLASH_MMC)
static int exynos_fastboot_alias_setup(void)
{
	struct blk_desc *blk_dev;
	struct disk_partition info = {0};
	char buf[128];
	int offset, i, j;

	blk_dev = blk_get_dev("mmc", CONFIG_FASTBOOT_FLASH_MMC_DEV);
	if (!blk_dev) {
		log_err("%s: required mmc device not available\n", __func__);
		return -ENODEV;
	}

	strcpy(buf, "fastboot_partition_alias_");
	offset = strlen(buf);

	for (i = 1; i < CONFIG_EFI_PARTITION_ENTRIES_NUMBERS; i++) {
		if (part_get_info(blk_dev, i, &info))
			continue;

		/*
		 * The partition name must be lowercase (stored in buf[]),
		 * as is expected in all fastboot partitions ...
		 */
		strlcpy(buf + offset, info.name, sizeof(buf) - offset);
		for (j = offset; buf[j]; j++)
			buf[j] = tolower(buf[j]);
		if (!strcmp(buf + offset, info.name))
			continue;
		/*
		 * ... However, if that isn't the case, a fastboot
		 * partition alias must be defined to establish it.
		 */
		env_set(buf, info.name);
	}

	return 0;
}
#else
/* Without eMMC flashing support there are no partitions to alias. */
static int exynos_fastboot_alias_setup(void)
{
	return 0;
}
#endif

static int exynos_fastboot_setup(void)
{
	phys_addr_t addr;

	/* Allocate and define buffer address for fastboot interface. */
	if (lmb_alloc(CONFIG_FASTBOOT_BUF_SIZE, &addr)) {
		log_err("%s: failed to allocate fastboot buffer\n", __func__);
		return -ENOMEM;
	}
	env_set_hex("fastboot_addr_r", addr);

	return exynos_fastboot_alias_setup();
}

int board_fdt_blob_setup(void **fdtp)
{
	/* If internal FDT is not available, use the external FDT instead. */
	if (fdt_check_header(*fdtp))
		*fdtp = (void *)get_prev_bl_fdt_addr();

	return 0;
}

int timer_init(void)
{
	ofnode timer_node;

	/*
	 * In a lot of Exynos devices, the previous bootloader does not
	 * set CNTFRQ_EL0 properly. However, the timer node in
	 * devicetree has the correct frequency, use that instead.
	 */
	timer_node = ofnode_by_compatible(ofnode_null(), "arm,armv8-timer");
	gd->arch.timer_rate_hz = ofnode_read_u32_default(timer_node,
							 "clock-frequency", 0);

	return 0;
}

int board_early_init_f(void)
{
	exynos_parse_dram_banks(gd->fdt_blob);

	return 0;
}

int dram_init(void)
{
	unsigned int i;

	/* Select the largest RAM bank for U-Boot. */
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		if (gd->ram_size < mem_map[i + 1].size) {
			gd->ram_base = mem_map[i + 1].phys;
			gd->ram_size = mem_map[i + 1].size;
		}
	}

	return 0;
}

int dram_init_banksize(void)
{
	unsigned int i;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		gd->dram[i].start = mem_map[i + 1].phys;
		gd->dram[i].size = mem_map[i + 1].size;
	}

	return 0;
}

/*
 * Some devices (e.g. Galaxy A3 2017) use a MIPI command-mode panel. S-BOOT
 * pushes the boot splash once and leaves DECON without a trigger source, so
 * nothing written to the framebuffer reaches the panel afterwards. Enable the
 * hardware (TE) trigger with auto mask, which makes DECON refresh the panel
 * on every TE pulse, as with a video-mode panel.
 */
#define EXYNOS7870_DECON_BASE			0x14830000UL
#define DECON_VIDCON0				0x0000
#define  DECON_VIDCON0_ENVID			BIT(1)
#define  DECON_VIDCON0_ENVID_F			BIT(0)
#define DECON_TRIGCON				0x06b0
#define  DECON_TRIGCON_SAVE_DISABLE_SYNCMGR	BIT(13)
#define  DECON_TRIGCON_HWTRIG_AUTO_MASK		BIT(6)
#define  DECON_TRIGCON_HWTRIGMASK_DISPIF0	BIT(4)
#define  DECON_TRIGCON_HWTRIGEN_I80_RGB		BIT(3)
#define  DECON_TRIGCON_SWTRIGEN_I80_RGB		BIT(0)
#define DECON_UPDATE				0x0710
#define  DECON_UPDATE_STANDALONE_F		BIT(0)

static void exynos7870_decon_enable_hw_trigger(void)
{
	void __iomem *base = (void __iomem *)EXYNOS7870_DECON_BASE;

	clrsetbits_le32(base + DECON_TRIGCON, DECON_TRIGCON_SWTRIGEN_I80_RGB,
			DECON_TRIGCON_HWTRIGEN_I80_RGB |
			DECON_TRIGCON_HWTRIG_AUTO_MASK |
			DECON_TRIGCON_SAVE_DISABLE_SYNCMGR |
			DECON_TRIGCON_HWTRIGMASK_DISPIF0);
	setbits_le32(base + DECON_VIDCON0,
		     DECON_VIDCON0_ENVID | DECON_VIDCON0_ENVID_F);
	setbits_le32(base + DECON_UPDATE, DECON_UPDATE_STANDALONE_F);
}

/*
 * The Galaxy A3 2017 routes USB D+/D- through the analog switch of its
 * S2MU004 MUIC, which the vendor kernel runs in manual mode. S-BOOT leaves the
 * switch open on a normal boot, so the USB gadget is never seen by the host.
 * Route D+/D- to the AP USB PHY.
 */
#define S2MU004_I2C_BUS_ADDR		0x13870000
#define S2MU004_I2C_ADDR		0x3d
#define S2MU004_REG_MUIC_ADC		0x61
#define S2MU004_REG_MUIC_DEVICE_TYPE1	0x62
#define S2MU004_REG_MUIC_CTRL1		0xc7
#define  S2MU004_CTRL1_SWITCH_OPEN	BIT(4)	/* 1: switches follow SW_CTRL */
#define  S2MU004_CTRL1_MANUAL_SW	BIT(2)	/* 0: manual switching */
#define S2MU004_REG_MUIC_SW_CTRL	0xca
#define  S2MU004_SW_CTRL_USB		(BIT(5) | BIT(2))	/* DM=USB, DP=USB */

static void a3y17lte_muic_route_usb(void)
{
	struct udevice *bus, *chip;
	int ctrl1, sw, ret;

	for (ret = uclass_first_device_err(UCLASS_I2C, &bus); !ret;
	     ret = uclass_next_device_err(&bus))
		if (dev_read_addr(bus) == S2MU004_I2C_BUS_ADDR)
			break;
	if (ret) {
		printf("MUIC: i2c bus @%x not found (%d)\n",
		       S2MU004_I2C_BUS_ADDR, ret);
		return;
	}

	ret = dm_i2c_probe(bus, S2MU004_I2C_ADDR, 0, &chip);
	if (ret) {
		printf("MUIC: s2mu004 not responding (%d)\n", ret);
		return;
	}

	ctrl1 = dm_i2c_reg_read(chip, S2MU004_REG_MUIC_CTRL1);
	sw = dm_i2c_reg_read(chip, S2MU004_REG_MUIC_SW_CTRL);
	printf("MUIC: before ctrl1=%02x sw_ctrl=%02x adc=%02x dev1=%02x\n",
	       ctrl1, sw, dm_i2c_reg_read(chip, S2MU004_REG_MUIC_ADC),
	       dm_i2c_reg_read(chip, S2MU004_REG_MUIC_DEVICE_TYPE1));
	if (ctrl1 < 0 || sw < 0)
		return;

	dm_i2c_reg_write(chip, S2MU004_REG_MUIC_SW_CTRL,
			 (sw & 0x03) | S2MU004_SW_CTRL_USB);
	dm_i2c_reg_write(chip, S2MU004_REG_MUIC_CTRL1,
			 (ctrl1 | S2MU004_CTRL1_SWITCH_OPEN) &
			 ~S2MU004_CTRL1_MANUAL_SW);

	printf("MUIC: after  ctrl1=%02x sw_ctrl=%02x\n",
	       dm_i2c_reg_read(chip, S2MU004_REG_MUIC_CTRL1),
	       dm_i2c_reg_read(chip, S2MU004_REG_MUIC_SW_CTRL));
}

int board_init(void)
{
	if (ofnode_device_is_compatible(ofnode_root(), "samsung,a3y17lte"))
		exynos7870_decon_enable_hw_trigger();

	return 0;
}

int misc_init_r(void)
{
	int ret;

	exynos_env_setup();

	if (ofnode_device_is_compatible(ofnode_root(), "samsung,a3y17lte"))
		a3y17lte_muic_route_usb();

	ret = exynos_blk_env_setup();
	if (ret)
		return ret;

	return exynos_fastboot_setup();
}
