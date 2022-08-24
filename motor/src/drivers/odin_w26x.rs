
pub mod odin {

type TransmitFunction = fn(&[u8]) -> Result<_, _>;

struct OdinW26X {

    // io abstraction
    transmit_function: TransmitFunction,
}

impl OdinW26X {
    fn init() -> OdinW26X {
        OdinW26X {

        }
    }

    fn set_transmit_function()
}

}