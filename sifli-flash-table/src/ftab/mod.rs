use std::mem::offset_of;

use crate::ptab;

pub(crate) mod structure;

const DFU_FLAG_AUTO: u16 = 2;

pub struct Ftab {
    pub(crate) structure: structure::SecConfiguration,
}

impl Ftab {
    pub fn new() -> Self {
        Self {
            structure: Default::default(),
        }
    }

    // Apply the partition table to the flash table
    pub fn apply(&mut self, table: &ptab::Ptab) {
        self.structure.ftab.secure_config.base = table.flash_table_info.start_addr;
        self.structure.ftab.secure_config.size = table.flash_table_info.size;

        self.structure.ftab.factory_calibration.base = table.flash_cal_info.start_addr;
        self.structure.ftab.factory_calibration.size = table.flash_cal_info.size;

        let used_hcpu = if let Some(hcpu) = table.hcpu_code_info.clone() {
            self.structure.ftab.hcpu.base = hcpu.start_addr;
            self.structure.ftab.hcpu.xip_base = hcpu.start_addr;
            // To be consist with ftab.c
            self.structure.ftab.hcpu.size = 0x200_000;
            // self.structure.ftab.hcpu.size = hcpu.size;

            self.structure.ftab.hcpu2.base = hcpu.start_addr;
            self.structure.ftab.hcpu2.xip_base = hcpu.start_addr;
            self.structure.ftab.hcpu2.size = 0x200_000;
            // self.structure.ftab.hcpu2.size = hcpu.size;
            true
        } else {
            false
        };

        let used_lcpu = if let Some(lcpu) = table.lcpu_code_info.clone() {
            self.structure.ftab.lcpu.base = lcpu.start_addr;
            self.structure.ftab.lcpu.xip_base = lcpu.start_addr;
            self.structure.ftab.lcpu.size = 0x200_000;
            // self.structure.ftab.lcpu.size = lcpu.size;

            self.structure.ftab.lcpu2.base = lcpu.start_addr;
            self.structure.ftab.lcpu2.xip_base = lcpu.start_addr;
            self.structure.ftab.lcpu2.size = 0x200_000;
            // self.structure.ftab.lcpu2.size = lcpu.size;
            true
        } else {
            false
        };

        // ftab.c: 
        // 5(boot): {.base = FLASH_BOOT_PATCH_START_ADDR, 
        //     .size = FLASH_BOOT_PATCH_SIZE, 
        //     .xip_base = BOOTLOADER_PATCH_CODE_ADDR, .flags = 0}
        // 9(boot2): {.base = BOOTLOADER_PATCH_CODE_ADDR, 
        //     .size = FLASH_BOOT_PATCH_SIZE, 
        //     .xip_base = BOOTLOADER_PATCH_CODE_ADDR, .flags = 0}
        self.structure.ftab.boot.base = table.bootloader_patch_flash_info.start_addr;
        self.structure.ftab.boot.xip_base = table.bootloader_patch_ram_info.start_addr;
        self.structure.ftab.boot.size = table.bootloader_patch_ram_info.size;

        self.structure.ftab.boot2.base = table.bootloader_patch_ram_info.start_addr;
        self.structure.ftab.boot2.xip_base = table.bootloader_patch_ram_info.start_addr;
        self.structure.ftab.boot2.size = table.bootloader_patch_ram_info.size;


        self.structure.ftab.bcpu.base = table.bootloader_info.start_addr;
        self.structure.ftab.bcpu.xip_base = table.bootloader_info.start_addr;
        self.structure.ftab.bcpu.size = 0x80_000;
        // self.structure.ftab.bcpu.size = table.bootloader_info.size;

        self.structure.ftab.bcpu2.base = table.bootloader_info.start_addr;
        self.structure.ftab.bcpu2.xip_base = table.bootloader_info.start_addr;
        self.structure.ftab.bcpu2.size = 0x80_000;
        // self.structure.ftab.bcpu2.size = table.bootloader_info.size;

        if used_hcpu {
            self.structure.imgs.hcpu.length = 200000;
            self.structure.imgs.hcpu.blksize = 512;
            self.structure.imgs.hcpu.flags = DFU_FLAG_AUTO;
        }
        else {
            self.structure.imgs.hcpu.length = 0xFFFFFFFF;
        }

        if used_lcpu {
            self.structure.imgs.lcpu.length = 200000;
            self.structure.imgs.lcpu.blksize = 512;
            self.structure.imgs.lcpu.flags = DFU_FLAG_AUTO;
        }
        else {
            self.structure.imgs.lcpu.length = 0xFFFFFFFF;
        }

        self.structure.imgs.lcpu.length = 0xFFFFFFFF;
        self.structure.imgs.bcpu.length = 0x80000;
        self.structure.imgs.bcpu.blksize = 512;
        self.structure.imgs.bcpu.flags = DFU_FLAG_AUTO;
        self.structure.imgs.boot.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu2.length = 0xFFFFFFFF;
        self.structure.imgs.bcpu2.length = 0xFFFFFFFF;
        self.structure.imgs.hcpu2.length = 0xFFFFFFFF;
        self.structure.imgs.boot2.length = 0xFFFFFFFF;
        self.structure.imgs.hcpu_ext2.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu_ext1.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu_ext2.length = 0xFFFFFFFF;
        self.structure.imgs.reserved.length = 0xFFFFFFFF;
        self.structure.imgs.single.length = 0xFFFFFFFF;

        self.structure.running_imgs.hcpu = if used_hcpu {
            (offset_of!(structure::SecConfiguration, imgs)
                + offset_of!(structure::Imgs, hcpu)) as u32
                + table.flash_table_info.start_addr
        }
        else {
            0xFFFFFFFF
        };

        self.structure.running_imgs.lcpu = if used_lcpu {
            (offset_of!(structure::SecConfiguration, imgs)
            + offset_of!(structure::Imgs, lcpu)) as u32
            + table.flash_table_info.start_addr
        }
        else {
            0xFFFFFFFF
        };
        
        self.structure.running_imgs.bl = (offset_of!(structure::SecConfiguration, imgs)
            + offset_of!(structure::Imgs, bcpu)) as u32
            + table.flash_table_info.start_addr
        
    }


    pub fn to_bytes(&self) -> &[u8] {
        unsafe {
            std::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                size_of::<Self>()
            )
        }
    }
}