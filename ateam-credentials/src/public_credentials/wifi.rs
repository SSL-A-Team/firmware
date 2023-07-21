use WifiCredential;

#[derive(Clone, Copy)]
pub enum WifiCredentialIndices {
    DummyField = 0,
    DummyDevelopmentNetwork = 1,
    COUNT = 2,
}

pub static wifi_credentials: [WifiCredential; WifiCredentialIndices::COUNT as usize] = [
    WifiCredential { ssid: "Dummy Field Network", password: "password1" },
    WifiCredential { ssid: "Dummy Development Network", password: "password2" },
];