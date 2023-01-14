/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "yportenv.h"

#include "yaffs_mtdif.h"

#include "linux/mtd/mtd.h"
#include "linux/types.h"
#include "linux/time.h"
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include "linux/mtd/nand.h"
#else
#include "linux/mtd/rawnand.h"
#endif
#include "linux/kernel.h"
#include "linux/version.h"
#include "linux/types.h"
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
#include "uapi/linux/major.h"
#endif

#include "yaffs_trace.h"
#include "yaffs_guts.h"
#include "yaffs_linux.h"
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0))
#define MTD_OPS_AUTO_OOB MTD_OOB_AUTO
#endif


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
#define mtd_erase(m, ei) (m)->erase(m, ei)
#define mtd_write_oob(m, addr, pops) (m)->write_oob(m, addr, pops)
#define mtd_read_oob(m, addr, pops) (m)->read_oob(m, addr, pops)
#define mtd_block_isbad(m, offs) (m)->block_isbad(m, offs)
#define mtd_block_markbad(m, offs) (m)->block_markbad(m, offs)
#endif

#define NOR_ERASESIZE	0x10000 // 64k
#define NOR_PAGE_SIZE	1024
#define NOR_OOB_SIZE	16
#define NOR_PAGE_OOB_SIZE	(NOR_PAGE_SIZE + NOR_OOB_SIZE)
#define NOR_CHUNKS_PER_BLOCK	(NOR_ERASESIZE / NOR_PAGE_OOB_SIZE)

static inline loff_t chunk_to_addr_nor(int nand_chunk)
{
        return NOR_ERASESIZE * (nand_chunk / NOR_CHUNKS_PER_BLOCK)
                + NOR_PAGE_OOB_SIZE
                * (nand_chunk % NOR_CHUNKS_PER_BLOCK);
}

/*
 * Sometimes m25p80/spi layer could return timeouts or failures.
 * We here retry reading until whole data is successfully read
 * because there is no ecc on spi flash and
 * return value of yaffs_rd_chunk_tags_nand() is not always checked,
 * and reading from spi flash should never fail anyway.
 */
//#define NOR_READ_VERIFY
//#define NOR_WRITE_VERIFY

#if defined(NOR_READ_VERIFY) || defined(NOR_WRITE_VERIFY)
static int mtd_read_verify(struct mtd_info *mtd, loff_t addr,
			   int data_len, u8 *data, int verify);

static int mtd_verify(struct mtd_info *mtd, loff_t addr,
		      int data_len, const u8 *data) {
	char buf[data_len];
	int retval = mtd_read_verify(mtd, addr, data_len, buf, 0);
	if (retval == 0 && memcmp(buf, data, data_len) != 0) {
		retval = -EPIPE;
	}
	return retval;
}
#endif

static int mtd_read_verify(struct mtd_info *mtd, loff_t addr,
			   int data_len, u8 *data, int verify) {
	int retval = -EIO;
	int tries;
	for (tries = 0; tries < 50 && retval != 0; ++tries) {
		size_t gotlen = 0;
		retval = mtd_read(mtd, addr, data_len, &gotlen, data);
		if (retval) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_read failed"
				    ", mtd error %d",
				    retval);
			continue;
		}

		if (gotlen != data_len) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_read failed"
				    ", gotlen %d != data_len %d\n",
				    gotlen, data_len);
			retval = -ENOSPC;
			continue;
		}
#ifdef NOR_READ_VERIFY
		if (verify) {
			retval = mtd_verify(mtd, addr, data_len, data);
			if (retval) {
				yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
					    "yaffs WARNING: mtd_read failed"
					    ", verify error %d",
					    retval);
				continue;
			}
		}
#endif
	}
	return retval;
}

static int mtd_write_verify(struct mtd_info *mtd, loff_t addr,
			    int data_len, const u8 *data) {
	int retval = -EIO;
	int tries;
	for (tries = 0; tries < 50 && retval != 0; ++tries) {
		size_t wrotelen = 0;
		retval = mtd_write(mtd, addr, data_len, &wrotelen, data);
		if (retval) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_write failed"
				    ", mtd error %d",
				    retval);
			continue;
		}

		if (wrotelen != data_len) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_write failed"
				    ", wrotelen %d != data_len %d\n",
				    wrotelen, data_len);
			retval = -ENOSPC;
			continue;
		}
#ifdef NOR_WRITE_VERIFY
		retval = mtd_verify(mtd, addr, data_len, data);
		if (retval) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_write failed"
				    ", verify error %d",
				    retval);
			continue;
		}
#endif
	}
	return retval;
}

static int yaffs_mtd_nor_write(struct yaffs_dev *dev, int nand_chunk,
			       const u8 *data, int data_len,
			       const u8 *oob, int oob_len)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	loff_t addr = chunk_to_addr_nor(nand_chunk);
	int retval = 0;

	yaffs_trace(YAFFS_TRACE_MTD,
		"yaffs_mtd_nor_write(%p, %d, %p, %d, %p, %d)\n",
		dev, nand_chunk, data, data_len, oob, oob_len);

	if (data && data_len) {
		retval = mtd_write_verify(mtd, addr, data_len, data);
	}
	if (oob && oob_len && retval == 0) {
		retval = mtd_write_verify(mtd, addr + NOR_PAGE_SIZE,
					  oob_len, oob);
	}
	if (retval) {
		yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
			"yaffs_mtd_nor_write failed, chunk %d, mtd error %d",
			nand_chunk, retval);
	}
	return retval ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_nor_read(struct yaffs_dev *dev, int nand_chunk,
			      u8 *data, int data_len,
			      u8 *oob, int oob_len,
			      enum yaffs_ecc_result *ecc_result)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	loff_t addr = chunk_to_addr_nor(nand_chunk);
	int retval = 0;

	if (data && data_len) {
		retval = mtd_read_verify(mtd, addr, data_len, data, 1);
	}
	if (oob && oob_len && retval == 0) {
		retval = mtd_read_verify(mtd, addr + NOR_PAGE_SIZE,
					 oob_len, oob, 1);
	}
	if (retval)
		yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
			"yaffs_mtd_nor_read failed, chunk %d, mtd error %d",
			nand_chunk, retval);

	if (ecc_result)
		*ecc_result = YAFFS_ECC_RESULT_NO_ERROR;

	return retval ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_nor_erase(struct yaffs_dev *dev, int block_no)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	int retval = -EIO;
	int tries;
	for (tries = 0; tries < 50 && retval != 0; ++tries) {
		struct erase_info ei;

		ei.addr = ((loff_t) block_no) * NOR_ERASESIZE;
		ei.len = NOR_ERASESIZE;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0))
		ei.mtd = mtd;
		ei.time = 1000;
		ei.retries = 2;
		ei.callback = NULL;
		ei.priv = (u_long) dev;
#endif
		retval = mtd_erase(mtd, &ei);

		if (retval) {
			yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
				    "yaffs WARNING: mtd_erase failed"
				    ", mtd error %d",
				    retval);
			continue;
		}
	}
	if (retval)
		yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
			"yaffs_mtd_nor_erase failed, block %d, mtd error %d",
			block_no, retval);
	return retval ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_nor_no_bad(struct yaffs_dev *dev, int block_no)
{
	return YAFFS_OK;
}

static int yaffs_mtd_write(struct yaffs_dev *dev, int nand_chunk,
			   const u8 *data, int data_len,
			   const u8 *oob, int oob_len)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	loff_t addr;
	struct mtd_oob_ops ops;
	int retval;

	yaffs_trace(YAFFS_TRACE_MTD,
		"yaffs_mtd_write(%p, %d, %p, %d, %p, %d)\n",
		dev, nand_chunk, data, data_len, oob, oob_len);

	if (!data || !data_len) {
		data = NULL;
		data_len = 0;
	}

	if (!oob || !oob_len) {
		oob = NULL;
		oob_len = 0;
	}

	addr = ((loff_t) nand_chunk) * dev->param.total_bytes_per_chunk;
	memset(&ops, 0, sizeof(ops));
	ops.mode = (mtd->oobsize == 16) ? MTD_OPS_PLACE_OOB : MTD_OPS_AUTO_OOB;
	ops.len = (data) ? data_len : 0;
	ops.ooblen = oob_len;
	ops.datbuf = (u8 *)data;
	ops.oobbuf = (u8 *)oob;

	retval = mtd_write_oob(mtd, addr, &ops);
	if (retval) {
		yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
			"write_oob failed, chunk %d, mtd error %d",
			nand_chunk, retval);
	}
	return retval ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_read(struct yaffs_dev *dev, int nand_chunk,
				   u8 *data, int data_len,
				   u8 *oob, int oob_len,
				   enum yaffs_ecc_result *ecc_result)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	loff_t addr;
	struct mtd_oob_ops ops;
	int retval;

	addr = ((loff_t) nand_chunk) * dev->param.total_bytes_per_chunk;
	memset(&ops, 0, sizeof(ops));
	ops.mode = (mtd->oobsize == 16) ? MTD_OPS_PLACE_OOB : MTD_OPS_AUTO_OOB;
	ops.len = (data) ? data_len : 0;
	ops.ooblen = oob_len;
	ops.datbuf = data;
	ops.oobbuf = oob;

#if (MTD_VERSION_CODE < MTD_VERSION(2, 6, 20))
	/* In MTD 2.6.18 to 2.6.19 nand_base.c:nand_do_read_oob() has a bug;
	 * help it out with ops.len = ops.ooblen when ops.datbuf == NULL.
	 */
	ops.len = (ops.datbuf) ? ops.len : ops.ooblen;
#endif
	/* Read page and oob using MTD.
	 * Check status and determine ECC result.
	 */
	retval = mtd_read_oob(mtd, addr, &ops);
	if (retval && retval != -EUCLEAN)
		yaffs_trace(YAFFS_TRACE_MTD | YAFFS_TRACE_ERROR,
			"read_oob failed, chunk %d, mtd error %d",
			nand_chunk, retval);

	switch (retval) {
	case 0:
		/* no error */
		if(ecc_result)
			*ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
		break;

	case -EUCLEAN:
		/* MTD's ECC fixed the data */
		if(ecc_result)
			*ecc_result = YAFFS_ECC_RESULT_FIXED;
		dev->n_ecc_fixed++;
		break;

	case -EBADMSG:
	default:
		/* MTD's ECC could not fix the data */
		dev->n_ecc_unfixed++;
		if(ecc_result)
			*ecc_result = YAFFS_ECC_RESULT_UNFIXED;
		return YAFFS_FAIL;
	}

	return YAFFS_OK;
}

static int yaffs_mtd_erase(struct yaffs_dev *dev, int block_no)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);

	loff_t addr;
	struct erase_info ei;
	int retval = 0;
	u32 block_size;

	block_size = dev->param.total_bytes_per_chunk *
		     dev->param.chunks_per_block;
	addr = ((loff_t) block_no) * block_size;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0))
	ei.mtd = mtd;
#endif
	ei.addr = addr;
	ei.len = block_size;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0))
	ei.time = 1000;
	ei.retries = 2;
	ei.callback = NULL;
	ei.priv = (u_long) dev;
#endif

	retval = mtd_erase(mtd, &ei);

	yaffs_trace(YAFFS_TRACE_MTD,
		"yaffs_mtd_erase(%p, %d, %d) = %d\n",
		    dev, block_no, block_size, retval);

	if (retval == 0)
		return YAFFS_OK;

	return YAFFS_FAIL;
}

static int yaffs_mtd_mark_bad(struct yaffs_dev *dev, int block_no)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	int blocksize = dev->param.chunks_per_block * dev->param.total_bytes_per_chunk;
	int retval;

	yaffs_trace(YAFFS_TRACE_BAD_BLOCKS, "marking block %d bad", block_no);

	retval = mtd_block_markbad(mtd, (loff_t) blocksize * block_no);
	return (retval) ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_check_bad(struct yaffs_dev *dev, int block_no)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	int blocksize = dev->param.chunks_per_block * dev->param.total_bytes_per_chunk;
	int retval;

	yaffs_trace(YAFFS_TRACE_MTD, "checking block %d bad", block_no);

	retval = mtd_block_isbad(mtd, (loff_t) blocksize * block_no);
	return (retval) ? YAFFS_FAIL : YAFFS_OK;
}

static int yaffs_mtd_initialise(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}

static int yaffs_mtd_deinitialise(struct yaffs_dev *dev)
{
	return YAFFS_OK;
}

void yaffs_mtd_drv_install(struct yaffs_dev *dev)
{
	struct mtd_info *mtd = yaffs_dev_to_mtd(dev);
	struct yaffs_driver *drv = &dev->drv;

	drv->drv_write_chunk_fn = yaffs_mtd_write;
	drv->drv_read_chunk_fn = yaffs_mtd_read;
	drv->drv_erase_fn = yaffs_mtd_erase;
	drv->drv_mark_bad_fn = yaffs_mtd_mark_bad;
	drv->drv_check_bad_fn = yaffs_mtd_check_bad;
	drv->drv_initialise_fn = yaffs_mtd_initialise;
	drv->drv_deinitialise_fn = yaffs_mtd_deinitialise;

	if (mtd->type == MTD_NORFLASH) {
		drv->drv_write_chunk_fn = yaffs_mtd_nor_write;
		drv->drv_read_chunk_fn = yaffs_mtd_nor_read;
		drv->drv_erase_fn = yaffs_mtd_nor_erase;
		drv->drv_mark_bad_fn = yaffs_mtd_nor_no_bad;
		drv->drv_check_bad_fn = yaffs_mtd_nor_no_bad;

		dev->param.no_tags_ecc = 1;
		dev->param.total_bytes_per_chunk = NOR_PAGE_SIZE;
		dev->param.chunks_per_block = NOR_CHUNKS_PER_BLOCK;
		dev->param.n_reserved_blocks = 2;
		dev->param.end_block = (uint32_t) mtd->size / NOR_ERASESIZE - 1;
		if (mtd->size < (32 << 20)) {
			dev->param.disable_summary = 1;
		}
		dev->param.disable_bad_block_marking = 1;
		dev->param.erase_is_slow = 1;
		/*
		 * Power off during NOR flash erase
		 * may lead to some bits at random places still not erased.
		 * And even erased bits may be unstable,
		 * as they may not have returned to stable "1" voltage.
		 * Workaround this by always_check_erased
		 * Proper fix would be to mark somewhere start and end of erase,
		 * and repeat the erase on mount, if no end mark is present.
		 *
		 * The same problem applies to power loss during write.
		 * There might be random bits wrong and unstable.
		 */
		dev->param.always_check_erased = 1;
#ifdef NOR_READ_VERIFY
		yaffs_trace(YAFFS_TRACE_ALWAYS,
			    "yaffs: MTD nor read verify");
#endif
#ifdef NOR_WRITE_VERIFY
		yaffs_trace(YAFFS_TRACE_ALWAYS,
			    "yaffs: MTD nor write verify");
#endif
	}
}

struct mtd_info * yaffs_get_mtd_device(dev_t sdev)
{
	struct mtd_info *mtd;

	mtd = yaffs_get_mtd_device(sdev);

	/* Check it's an mtd device..... */
	if (MAJOR(sdev) != MTD_BLOCK_MAJOR)
		return NULL;	/* This isn't an mtd device */

	/* Check it's NAND */
	if (mtd->type != MTD_NANDFLASH) {
		yaffs_trace(YAFFS_TRACE_ALWAYS,
			"yaffs: MTD device is not NAND it's type %d",
			mtd->type);
		return NULL;
	}

	yaffs_trace(YAFFS_TRACE_OS, " %s %d", WRITE_SIZE_STR, WRITE_SIZE(mtd));
	yaffs_trace(YAFFS_TRACE_OS, " oobsize %d", mtd->oobsize);
	yaffs_trace(YAFFS_TRACE_OS, " erasesize %d", mtd->erasesize);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
	yaffs_trace(YAFFS_TRACE_OS, " size %u", mtd->size);
#else
	yaffs_trace(YAFFS_TRACE_OS, " size %lld", mtd->size);
#endif

	return mtd;
}

int yaffs_verify_mtd(struct mtd_info *mtd, int yaffs_version, int inband_tags)
{
	if (yaffs_version == 2) {
		if ((WRITE_SIZE(mtd) < YAFFS_MIN_YAFFS2_CHUNK_SIZE ||
		     mtd->oobsize < YAFFS_MIN_YAFFS2_SPARE_SIZE) &&
		    mtd->type != MTD_NORFLASH &&
		    !inband_tags) {
			yaffs_trace(YAFFS_TRACE_ALWAYS,
				"MTD device does not have the right page sizes"
			);
			return -1;
		}
	} else {
		if (WRITE_SIZE(mtd) < YAFFS_BYTES_PER_CHUNK ||
		    mtd->oobsize != YAFFS_BYTES_PER_SPARE) {
			yaffs_trace(YAFFS_TRACE_ALWAYS,
				"MTD device does not support have the right page sizes"
			);
			return -1;
		}
	}

	return 0;
}

void yaffs_put_mtd_device(struct mtd_info *mtd)
{
	if (mtd)
		put_mtd_device(mtd);
}
