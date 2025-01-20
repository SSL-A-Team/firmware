struct GitStatus {
    git_struct_id: u32,
    git_hash: u32,
    git_dirty: u32,
}
const GIT_STATUS: GitStatus = GitStatus {git_struct_id: 0xAABBCCDD, git_hash: 0, git_dirty: 0};

pub fn get_git_id() -> u32 {
    GIT_STATUS.git_struct_id
}
pub fn get_git_hash() -> u32 {
    GIT_STATUS.git_hash
}
pub fn get_git_dirty() -> u32 {
    GIT_STATUS.git_dirty
}