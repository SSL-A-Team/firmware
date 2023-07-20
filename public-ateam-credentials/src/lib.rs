#![no_std]
#![allow(non_upper_case_globals)]

#[derive(Clone, Copy)]
pub enum WifiCredentialIndices {
    DummyField = 0,
    DummyDevelopmentNetwork = 1,
    COUNT = 2,
}

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

pub static wifi_credentials: [WifiCredential; WifiCredentialIndices::COUNT as usize] = [
    WifiCredential { ssid: "Dummy Field Network", password: "password1" },
    WifiCredential { ssid: "Dummy Development Network", password: "password2" },
];
