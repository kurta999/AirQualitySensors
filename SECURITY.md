# Security policy

## Reporting a vulnerability

Do not publish credentials, private network details, exploit instructions, or an unpatched vulnerability in a public issue. Use GitHub's private vulnerability reporting feature from the repository's **Security** tab when it is available. If it is unavailable, contact the repository owner privately through the contact method on their GitHub profile before disclosing details.

Include the affected commit, configuration, impact, reproduction conditions, and a minimal proposed fix if possible. Remove real SSIDs, passwords, server addresses, device identifiers, and captured user data from logs.

## Security boundaries

- Wi-Fi is disabled by default.
- Local credentials are excluded through `.gitignore`, but they are compiled into firmware when enabled and can be recovered from a device image.
- The ESP32 transport uses unauthenticated plain TCP. Treat it as trusted-LAN functionality, not an Internet-facing service.
- Remote reset is disabled by default.
- USB keyboards are untrusted input devices; the gateway forwards HID state but does not authenticate the device.
- Physical access, SWD access, malicious replacement firmware, and compromised sensor modules are outside the software's protection boundary.

Rotate any credential immediately if it was ever committed, shared in a log, or flashed to a device that left your control. Removing it from the latest source does not remove it from Git history or existing firmware images.
