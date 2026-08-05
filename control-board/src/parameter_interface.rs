use ateam_common_packets::{ParameterCommand, ParameterName};

pub trait ParameterInterface {
    fn processes_cmd(&self, param_cmd: &ParameterCommand) -> bool;

    fn has_name(&self, param_name: ParameterName) -> bool;

    fn apply_command(
        &mut self,
        param_cmd: &ParameterCommand,
    ) -> Result<ParameterCommand, ParameterCommand>;
}
