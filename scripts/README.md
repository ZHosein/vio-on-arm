# GitHub App Token Generation Scripts

This directory contains helper scripts to generate installation access tokens for the GitHub App.

## Available Scripts

### 1. generate-app-token.sh (Bash)

A bash script that uses OpenSSL and curl to generate tokens.

**Requirements:**
- `bash`
- `openssl`
- `curl`

**Usage:**
```bash
./generate-app-token.sh <app_id> <installation_id> <private_key_path>
```

**Example:**
```bash
./generate-app-token.sh 123456 789012 ./github-app-key.pem
```

### 2. generate-app-token.py (Python)

A Python script that uses PyJWT and requests libraries to generate tokens.

**Requirements:**
- Python 3.6+
- `PyJWT` library
- `cryptography` library
- `requests` library

**Installation:**
```bash
pip install PyJWT cryptography requests
```

**Usage:**
```bash
python generate-app-token.py <app_id> <installation_id> <private_key_path>
```

**Example:**
```bash
python generate-app-token.py 123456 789012 ./github-app-key.pem
```

## Getting Your Credentials

To use these scripts, you need three pieces of information from the GitHub App:

1. **App ID**: Found in your GitHub App settings under "About"
2. **Installation ID**: 
   - Go to your GitHub App settings
   - Click on "Install App" in the sidebar
   - Click on the installation (repository)
   - The installation ID is in the URL: `https://github.com/settings/installations/[INSTALLATION_ID]`
3. **Private Key**: Generated and downloaded from your GitHub App settings

## Token Details

- **Validity**: Installation access tokens are valid for 1 hour
- **Permissions**: Tokens inherit the permissions configured in your GitHub App
- **Scope**: Tokens are scoped to the repositories where the app is installed

## Using the Token

Once generated, you can use the token for:

### Git Operations
```bash
# Clone
git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git

# Add remote
git remote add origin https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git
```

### Docker Registry
```bash
# Login
echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin

# Pull image
docker pull ghcr.io/zhosein/vio-on-arm:latest
```

### Environment Variable
```bash
# Set for current session
export GITHUB_TOKEN=TOKEN

# Use with git
git clone https://x-access-token:$GITHUB_TOKEN@github.com/ZHosein/vio-on-arm.git
```

## Troubleshooting

### "command not found" errors
Make sure the scripts are executable:
```bash
chmod +x generate-app-token.sh
chmod +x generate-app-token.py
```

### "Error: Required libraries not installed" (Python)
Install the required Python packages:
```bash
pip install PyJWT cryptography requests
```

### "Error: Failed to generate token"
- Verify your App ID is correct
- Verify your Installation ID is correct
- Ensure your private key file is valid and readable
- Check that your GitHub App has the necessary permissions

## Security Notes

- Never commit private keys to version control
- Keep your private keys secure
- Rotate keys periodically
- Use different apps/tokens for different environments
- Tokens expire after 1 hour - regenerate as needed
