// Build-script helper for firmware images built as ELF files (the kicker
// board), pulled in with `include!` after util/fw_image_build.rs. Needs the
// `object` crate as a build dependency.

/// Like `prepare_fw_image`, but reads the ELF `elf` and converts it to the flat
/// binary that gets flashed.
fn prepare_fw_image_from_elf(
    elf: &std::path::Path,
    out_name: &str,
    self_hash_magic: &[u8; IMG_HASH_MAGIC_LEN],
) -> Option<[u8; IMG_HASH_LEN]> {
    println!("cargo:rerun-if-changed={}", elf.display());
    let Ok(data) = std::fs::read(elf) else {
        warn_missing_fw_image(elf, out_name);
        return None;
    };
    let image = elf_to_flat_binary(&data)
        .unwrap_or_else(|e| panic!("failed to convert '{}' to a flat binary: {e}", elf.display()));
    Some(embed_fw_image(image, elf, out_name, self_hash_magic))
}

/// Equivalent of `objcopy -O binary`: the contents of every allocated section
/// that occupies space in the file, placed by load address (LMA) relative to
/// the lowest one, with gaps zero-filled.
fn elf_to_flat_binary(data: &[u8]) -> Result<Vec<u8>, object::read::Error> {
    use object::elf::{FileHeader32, PT_LOAD, SHF_ALLOC, SHT_NOBITS};
    use object::read::elf::{FileHeader, ProgramHeader, SectionHeader};
    use object::Endianness;

    let header = FileHeader32::<Endianness>::parse(data)?;
    let endian = header.endian()?;
    let segments = header.program_headers(endian, data)?;

    let mut chunks: Vec<(u64, &[u8])> = Vec::new();
    for section in header.section_headers(endian, data)? {
        let size = section.sh_size(endian) as u64;
        if section.sh_flags(endian).0 & SHF_ALLOC.0 == 0
            || section.sh_type(endian) == SHT_NOBITS
            || size == 0
        {
            continue;
        }
        // A section's LMA comes from the load segment containing it (e.g.
        // .data runs from RAM but is stored in flash).
        let offset = section.sh_offset(endian) as u64;
        let lma = segments
            .iter()
            .find(|p| {
                let p_offset = p.p_offset(endian) as u64;
                p.p_type(endian) == PT_LOAD
                    && offset >= p_offset
                    && offset + size <= p_offset + p.p_filesz(endian) as u64
            })
            .map(|p| p.p_paddr(endian) as u64 + (offset - p.p_offset(endian) as u64))
            .unwrap_or(section.sh_addr(endian) as u64);
        chunks.push((lma, section.data(endian, data)?));
    }

    let Some(base) = chunks.iter().map(|(lma, _)| *lma).min() else {
        return Ok(Vec::new());
    };
    let end = chunks.iter().map(|(lma, d)| lma + d.len() as u64).max().unwrap();
    let mut image = vec![0u8; (end - base) as usize];
    for (lma, d) in chunks {
        let start = (lma - base) as usize;
        image[start..start + d.len()].copy_from_slice(d);
    }
    Ok(image)
}
