# Security policy

## Do not publish secrets

Never commit Wi-Fi credentials, `.env` files, private keys, API tokens, device
allowlists, personal photos, face embeddings or production server addresses.

## Camera and MCP exposure

Camera capture and MCP endpoints can control physical hardware and access image
data. Deploy them only behind authentication and TLS. Avoid exposing development
ports directly to the public Internet.

## Reporting

Please report security issues privately to the repository maintainer rather
than opening a public issue containing credentials, device identifiers or
reproduction photos.
