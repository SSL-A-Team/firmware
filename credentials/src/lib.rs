#![no_std]
#![allow(non_upper_case_globals)]

#[cfg(not(feature = "no-private-credentials"))]
pub mod private_credentials;
pub mod public_credentials;

#[derive(Clone, Copy)]
pub struct WifiCredential {
    ssid: &'static str,
    password: &'static str,
}

impl WifiCredential {
    pub fn get_ssid(&self) -> &'static str {
        self.ssid
    }

    pub fn get_password(&self) -> &'static str {
        self.password
    }
}
