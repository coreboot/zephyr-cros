/*
 * Copyright (c) 2023 Intel Corporation
 * Copyright (c) 2024 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/llext/elf.h>
#include <zephyr/llext/loader.h>
#include <zephyr/llext/llext.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(llext, CONFIG_LLEXT_LOG_LEVEL);

#include <string.h>

#include "llext_priv.h"

#ifdef CONFIG_LLEXT_EXPORT_BUILTINS_BY_SLID
#define SYM_NAME_OR_SLID(name, slid) ((const char *) slid)
#else
#define SYM_NAME_OR_SLID(name, slid) name
#endif

__weak int arch_elf_relocate(elf_rela_t *rel, uintptr_t loc,
			     uintptr_t sym_base_addr, const char *sym_name, uintptr_t load_bias)
{
	return -EOPNOTSUPP;
}

__weak void arch_elf_relocate_local(struct llext_loader *ldr, struct llext *ext,
				    const elf_rela_t *rel, const elf_sym_t *sym, size_t got_offset)
{
}

/*
 * Find the section containing the supplied offset and return file offset for
 * that value
 */
static size_t llext_file_offset(struct llext_loader *ldr, size_t offset)
{
	unsigned int i;

	for (i = 0; i < LLEXT_MEM_COUNT; i++)
		if (ldr->sects[i].sh_addr <= offset &&
		    ldr->sects[i].sh_addr + ldr->sects[i].sh_size > offset)
			return offset - ldr->sects[i].sh_addr + ldr->sects[i].sh_offset;

	return offset;
}

static void llext_link_plt(struct llext_loader *ldr, struct llext *ext,
			   elf_shdr_t *shdr, bool do_local, elf_shdr_t *tgt)
{
	unsigned int sh_cnt = shdr->sh_size / shdr->sh_entsize;
	/*
	 * CPU address where the .text section is stored, we use .text just as a
	 * reference point
	 */
	uint8_t *text = ext->mem[LLEXT_MEM_TEXT];

	LOG_DBG("Found %p in PLT %u size %zu cnt %u text %p",
		(void *)llext_string(ldr, ext, LLEXT_MEM_SHSTRTAB, shdr->sh_name),
		shdr->sh_type, (size_t)shdr->sh_entsize, sh_cnt, (void *)text);

	const elf_shdr_t *sym_shdr = ldr->sects + LLEXT_MEM_SYMTAB;
	unsigned int sym_cnt = sym_shdr->sh_size / sym_shdr->sh_entsize;

	for (unsigned int i = 0; i < sh_cnt; i++) {
		elf_rela_t rela;

		int ret = llext_seek(ldr, shdr->sh_offset + i * shdr->sh_entsize);

		if (!ret) {
			ret = llext_read(ldr, &rela, sizeof(rela));
		}

		if (ret < 0) {
			LOG_ERR("PLT: failed to read RELA #%u, trying to continue", i);
			continue;
		}

		/* Index in the symbol table */
		unsigned int j = ELF32_R_SYM(rela.r_info);

		if (j >= sym_cnt) {
			LOG_WRN("PLT: idx %u >= %u", j, sym_cnt);
			continue;
		}

		elf_sym_t sym_tbl;

		ret = llext_seek(ldr, sym_shdr->sh_offset + j * sizeof(elf_sym_t));
		if (!ret) {
			ret = llext_read(ldr, &sym_tbl, sizeof(sym_tbl));
		}

		if (ret < 0) {
			LOG_ERR("PLT: failed to read symbol table #%u RELA #%u, trying to continue",
				j, i);
			continue;
		}

		uint32_t stt = ELF_ST_TYPE(sym_tbl.st_info);

		if (stt != STT_FUNC &&
		    stt != STT_SECTION &&
		    stt != STT_OBJECT &&
		    (stt != STT_NOTYPE || sym_tbl.st_shndx != SHN_UNDEF)) {
			continue;
		}

		const char *name = llext_string(ldr, ext, LLEXT_MEM_STRTAB, sym_tbl.st_name);

		/*
		 * Both r_offset and sh_addr are addresses for which the extension
		 * has been built.
		 */
		size_t got_offset;

		if (tgt) {
			got_offset = rela.r_offset + tgt->sh_offset -
				ldr->sects[LLEXT_MEM_TEXT].sh_offset;
		} else {
			got_offset = llext_file_offset(ldr, rela.r_offset) -
				ldr->sects[LLEXT_MEM_TEXT].sh_offset;
		}

		uint32_t stb = ELF_ST_BIND(sym_tbl.st_info);
		const void *link_addr;

		switch (stb) {
		case STB_GLOBAL:
			link_addr = llext_find_sym(NULL,
				SYM_NAME_OR_SLID(name, sym_tbl.st_value));

			if (!link_addr)
				link_addr = llext_find_sym(&ext->sym_tab, name);

			if (!link_addr) {
				LOG_WRN("PLT: cannot find idx %u name %s", j, name);
				continue;
			}

			/* Resolve the symbol */
			*(const void **)(text + got_offset) = link_addr;
			break;
		case STB_LOCAL:
			if (do_local) {
				arch_elf_relocate_local(ldr, ext, &rela, &sym_tbl, got_offset);
			}
		}

		LOG_DBG("symbol %s offset %#zx r-offset %#zx .text offset %#zx stb %u",
			name, got_offset,
			(size_t)rela.r_offset, (size_t)ldr->sects[LLEXT_MEM_TEXT].sh_offset, stb);
	}
}

int llext_link(struct llext_loader *ldr, struct llext *ext, bool do_local)
{
	uintptr_t loc = 0;
	elf_shdr_t shdr;
	elf_rela_t rel;
	elf_sym_t sym;
	elf_word rel_cnt = 0;
	const char *name;
	int i, ret;
	size_t pos;

	for (i = 0, pos = ldr->hdr.e_shoff;
	     i < ldr->hdr.e_shnum - 1;
	     i++, pos += ldr->hdr.e_shentsize) {
		ret = llext_seek(ldr, pos);
		if (ret != 0) {
			return ret;
		}

		ret = llext_read(ldr, &shdr, sizeof(elf_shdr_t));
		if (ret != 0) {
			return ret;
		}

		/* find relocation sections */
		if (shdr.sh_type != SHT_REL && shdr.sh_type != SHT_RELA) {
			continue;
		}

		rel_cnt = shdr.sh_size / shdr.sh_entsize;

		name = llext_string(ldr, ext, LLEXT_MEM_SHSTRTAB, shdr.sh_name);

		if (strcmp(name, ".rel.text") == 0) {
			loc = (uintptr_t)ext->mem[LLEXT_MEM_TEXT];
		} else if (strcmp(name, ".rel.bss") == 0 ||
			   strcmp(name, ".rela.bss") == 0) {
			loc = (uintptr_t)ext->mem[LLEXT_MEM_BSS];
		} else if (strcmp(name, ".rel.rodata") == 0 ||
			   strcmp(name, ".rela.rodata") == 0) {
			loc = (uintptr_t)ext->mem[LLEXT_MEM_RODATA];
		} else if (strcmp(name, ".rel.data") == 0) {
			loc = (uintptr_t)ext->mem[LLEXT_MEM_DATA];
		} else if (strcmp(name, ".rel.exported_sym") == 0) {
			loc = (uintptr_t)ext->mem[LLEXT_MEM_EXPORT];
		} else if (strcmp(name, ".rela.plt") == 0 ||
			   strcmp(name, ".rela.dyn") == 0) {
			llext_link_plt(ldr, ext, &shdr, do_local, NULL);
			continue;
		} else if (strncmp(name, ".rela", 5) == 0 && strlen(name) > 5) {
			elf_shdr_t *tgt = llext_section_by_name(ldr, name + 5);

			if (tgt)
				llext_link_plt(ldr, ext, &shdr, do_local, tgt);
			continue;
		} else if (strcmp(name, ".rel.dyn") == 0) {
			/* we assume that first load segment starts at MEM_TEXT */
			loc = (uintptr_t)ext->mem[LLEXT_MEM_TEXT];
		}

		LOG_DBG("relocation section %s (%d) linked to section %d has %zd relocations",
			name, i, shdr.sh_link, (size_t)rel_cnt);

		for (int j = 0; j < rel_cnt; j++) {
			/* get each relocation entry */
			ret = llext_seek(ldr, shdr.sh_offset + j * shdr.sh_entsize);
			if (ret != 0) {
				return ret;
			}

			ret = llext_read(ldr, &rel, shdr.sh_entsize);
			if (ret != 0) {
				return ret;
			}

			/* get corresponding symbol */
			ret = llext_seek(ldr, ldr->sects[LLEXT_MEM_SYMTAB].sh_offset
				    + ELF_R_SYM(rel.r_info) * sizeof(elf_sym_t));
			if (ret != 0) {
				return ret;
			}

			ret = llext_read(ldr, &sym, sizeof(elf_sym_t));
			if (ret != 0) {
				return ret;
			}

			name = llext_string(ldr, ext, LLEXT_MEM_STRTAB, sym.st_name);

			LOG_DBG("relocation %d:%d info %zx (type %zd, sym %zd) offset %zd sym_name "
				"%s sym_type %d sym_bind %d sym_ndx %d",
				i, j, (size_t)rel.r_info, (size_t)ELF_R_TYPE(rel.r_info),
				(size_t)ELF_R_SYM(rel.r_info),
				(size_t)rel.r_offset, name, ELF_ST_TYPE(sym.st_info),
				ELF_ST_BIND(sym.st_info), sym.st_shndx);

			uintptr_t link_addr, op_loc;

			op_loc = loc + rel.r_offset;

			if (ELF_R_SYM(rel.r_info) == 0) {
				/* no symbol ex: R_ARM_V4BX relocation, R_ARM_RELATIVE  */
				link_addr = 0;
			} else if (sym.st_shndx == SHN_UNDEF) {
				/* If symbol is undefined, then we need to look it up */
				link_addr = (uintptr_t)llext_find_sym(NULL,
					SYM_NAME_OR_SLID(name, sym.st_value));

				if (link_addr == 0) {
					LOG_ERR("Undefined symbol with no entry in "
						"symbol table %s, offset %zd, link section %d",
						name, (size_t)rel.r_offset, shdr.sh_link);
					return -ENODATA;
				}

				LOG_INF("found symbol %s at 0x%lx", name, link_addr);
			} else if (sym.st_shndx == SHN_ABS) {
				/* Absolute symbol */
				link_addr = sym.st_value;
			} else if ((sym.st_shndx < ldr->hdr.e_shnum) &&
				!IN_RANGE(sym.st_shndx, SHN_LORESERVE, SHN_HIRESERVE)) {
				/* This check rejects all relocations whose target symbol
				 * has a section index higher than the maximum possible
				 * in this ELF file, or belongs in the reserved range:
				 * they will be caught by the `else` below and cause an
				 * error to be returned. This aborts the LLEXT's loading
				 * and prevents execution of improperly relocated code,
				 * which is dangerous.
				 *
				 * Note that the unsupported SHN_COMMON section is rejected
				 * as part of this check. Also note that SHN_ABS would be
				 * rejected as well, but we want to handle it properly:
				 * for this reason, this check must come AFTER handling
				 * the case where the symbol's section index is SHN_ABS!
				 *
				 *
				 * For regular symbols, the link address is obtained by
				 * adding st_value to the start address of the section
				 * in which the target symbol resides.
				 */
				link_addr = (uintptr_t)ext->mem[ldr->sect_map[sym.st_shndx]]
					+ sym.st_value;
			} else {
				LOG_ERR("rela section %d, entry %d: cannot apply relocation: "
					"target symbol has unexpected section index %d (0x%X)",
					i, j, sym.st_shndx, sym.st_shndx);
				return -ENOEXEC;
			}

			LOG_INF("writing relocation symbol %s type %zd sym %zd at addr 0x%lx "
				"addr 0x%lx",
				name, (size_t)ELF_R_TYPE(rel.r_info), (size_t)ELF_R_SYM(rel.r_info),
				op_loc, link_addr);

			/* relocation */
			ret = arch_elf_relocate(&rel, op_loc, link_addr, name,
					     (uintptr_t)ext->mem[LLEXT_MEM_TEXT]);
			if (ret != 0) {
				return ret;
			}
		}
	}

#ifdef CONFIG_CACHE_MANAGEMENT
	/* Make sure changes to ext sections are flushed to RAM */
	for (i = 0; i < LLEXT_MEM_COUNT; ++i) {
		if (ext->mem[i]) {
			sys_cache_data_flush_range(ext->mem[i], ext->mem_size[i]);
			sys_cache_instr_invd_range(ext->mem[i], ext->mem_size[i]);
		}
	}
#endif

	return 0;
}
