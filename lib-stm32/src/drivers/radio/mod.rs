pub mod at_protocol;
pub mod edm_protocol;
pub mod odin_w26x;

#[derive(Debug)]
pub enum RadioError {
    SendCommandLowLevelBufferFull,
    SendCommandInvalidCommunicationMode,
}