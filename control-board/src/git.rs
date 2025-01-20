use core::ptr::read_volatile;

struct GitStatus {
    git_struct_id: u32,
    git_hash: u32,
    git_dirty: u32,
}
static mut GIT_STATUS: GitStatus = GitStatus {git_struct_id: 0x6789ABCD, git_hash: 0, git_dirty: 0};

pub fn get_git_id() -> u32 {
    // Enforce a read from memory, can get optimized out otherwise
    unsafe {
        return read_volatile(&GIT_STATUS.git_struct_id as *const u32);
    }
}

pub fn get_git_hash() -> u32 {
    // Enforce a read from memory, can get optimized out otherwise
    unsafe {
        return read_volatile(&GIT_STATUS.git_hash as *const u32);
    }
}

pub fn get_git_dirty() -> u32 {
    // Enforce a read from memory, can get optimized out otherwise
    unsafe {
        return read_volatile(&GIT_STATUS.git_dirty as *const u32);
    }
}