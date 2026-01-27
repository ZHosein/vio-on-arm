# Troubleshooting Guide

Common issues and solutions when accessing the vio-on-arm repository and Docker images.

## Table of Contents
- [Git/Repository Issues](#gitrepository-issues)
- [Docker/Package Issues](#dockerpackage-issues)
- [Token Generation Issues](#token-generation-issues)
- [SSH Issues](#ssh-issues)
- [General Authentication Issues](#general-authentication-issues)

---

## Git/Repository Issues

### Issue: `repository not found` or `404 Not Found`

**Possible Causes:**
1. Repository is private and you don't have access
2. Token doesn't have correct permissions
3. Token has expired
4. Incorrect repository URL

**Solutions:**
```bash
# Verify repository URL
# Correct format: https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git

# Check token hasn't expired
# GitHub App tokens expire after 1 hour
# Regenerate if needed
./scripts/generate-app-token.sh APP_ID INSTALL_ID key.pem

# For PAT, check expiration in GitHub settings
# Settings → Developer settings → Personal access tokens
```

### Issue: `Authentication failed`

**Solutions:**
```bash
# Make sure token is properly formatted in URL
git clone https://x-access-token:YOUR_TOKEN_HERE@github.com/ZHosein/vio-on-arm.git

# NOT like this (common mistake):
# git clone https://YOUR_TOKEN_HERE@github.com/ZHosein/vio-on-arm.git
#                      ↑ missing x-access-token: prefix

# Clear any cached credentials
git credential-cache exit

# On macOS, clear keychain
git credential-osxkeychain erase
host=github.com
protocol=https
# Press Enter twice
```

### Issue: Git keeps asking for password

**Solutions:**
```bash
# Make sure you're using HTTPS with token, not SSH
# Correct:
git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git

# Or set up credential helper
git config --global credential.helper cache

# For longer cache (1 hour):
git config --global credential.helper 'cache --timeout=3600'
```

---

## Docker/Package Issues

### Issue: `unauthorized: authentication required`

**Possible Causes:**
1. Not logged into GitHub Container Registry
2. Token expired
3. Token doesn't have `packages:read` permission

**Solutions:**
```bash
# Make sure you're logged in
docker login ghcr.io

# Or login with token explicitly
echo YOUR_TOKEN | docker login ghcr.io -u x-access-token --password-stdin

# For PAT, use your GitHub username:
echo YOUR_TOKEN | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin

# Verify login
docker system info | grep -A 3 "Registry"
```

### Issue: `denied: permission_denied` or `requested access to the resource is denied`

**Possible Causes:**
1. Package is private and token doesn't have access
2. Wrong package name
3. Token permissions insufficient

**Solutions:**
```bash
# Verify package name (case-sensitive!)
# Correct: ghcr.io/zhosein/vio-on-arm:latest
# Note: username is lowercase!

# Check token has packages:read permission
# For GitHub App: Check app settings
# For PAT: Check token permissions in GitHub settings

# Try pulling with explicit authentication
docker logout ghcr.io
echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin
docker pull ghcr.io/zhosein/vio-on-arm:latest
```

### Issue: `Error response from daemon: Get https://ghcr.io/v2/: denied`

**Solutions:**
```bash
# Clear Docker credentials
docker logout ghcr.io

# Remove stored credentials
rm -f ~/.docker/config.json

# Login again
echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin

# Try pull again
docker pull ghcr.io/zhosein/vio-on-arm:latest
```

### Issue: `manifest unknown: manifest unknown`

**Possible Causes:**
1. Image tag doesn't exist
2. Image hasn't been built yet

**Solutions:**
```bash
# List available tags (if you have access to repo)
git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git
cd vio-on-arm
git tag

# Or check GitHub Container Registry page
# https://github.com/users/ZHosein/packages/container/vio-on-arm/versions

# Common tags:
docker pull ghcr.io/zhosein/vio-on-arm:latest
docker pull ghcr.io/zhosein/vio-on-arm:main
docker pull ghcr.io/zhosein/vio-on-arm:v1.0.0
```

---

## Token Generation Issues

### Issue: `Error: openssl is required but not installed` (Bash script)

**Solutions:**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install openssl curl

# macOS
brew install openssl curl

# Or use Python script instead
pip install PyJWT cryptography requests
python3 scripts/generate-app-token.py APP_ID INSTALL_ID key.pem
```

### Issue: `Error: Required libraries not installed` (Python script)

**Solutions:**
```bash
# Install required packages
pip install PyJWT cryptography requests

# Or use requirements file
pip install -r scripts/requirements.txt

# If pip is not installed
# Ubuntu/Debian:
sudo apt-get install python3-pip

# macOS:
brew install python3

# Or use Bash script instead
./scripts/generate-app-token.sh APP_ID INSTALL_ID key.pem
```

### Issue: `Error: Private key file not found`

**Solutions:**
```bash
# Verify file path
ls -la /path/to/private-key.pem

# Use absolute path
./scripts/generate-app-token.sh APP_ID INSTALL_ID /absolute/path/to/key.pem

# Or relative path from script location
cd vio-on-arm
./scripts/generate-app-token.sh APP_ID INSTALL_ID ./my-key.pem
```

### Issue: `Error: Failed to generate token` with GitHub API error

**Possible Causes:**
1. Incorrect App ID
2. Incorrect Installation ID
3. Invalid private key
4. App not installed on repository

**Solutions:**
```bash
# Verify App ID
# Found in: GitHub Settings → Developer settings → GitHub Apps → [Your App] → About

# Verify Installation ID
# Go to: GitHub Settings → Developer settings → GitHub Apps → [Your App] → Install App
# Click on installation, ID is in URL: /settings/installations/INSTALLATION_ID

# Verify private key format
head -1 /path/to/key.pem
# Should show: -----BEGIN RSA PRIVATE KEY-----

# Check app is installed
# GitHub Settings → Developer settings → GitHub Apps → [Your App] → Install App
# Should show installation for vio-on-arm repository
```

### Issue: Token works for git but not Docker

**Solutions:**
```bash
# GitHub App tokens need both contents AND packages permissions
# Check app permissions in GitHub Settings

# Reinstall app with correct permissions
# GitHub Settings → Developer settings → GitHub Apps → [Your App]
# → Edit → Permissions → Add "Packages: Read"
# Then reinstall app on repository

# Or use Personal Access Token with both permissions
```

---

## SSH Issues

### Issue: `Permission denied (publickey)`

**Solutions:**
```bash
# Verify your public key was added as deploy key
# Repository Settings → Deploy keys → Should see your key

# Test SSH connection
ssh -T git@github.com

# If using custom SSH config, test with specific host
ssh -T github.com-vio-on-arm

# Check SSH key permissions (must be 600 or 400)
chmod 600 ~/.ssh/vio-on-arm
ls -la ~/.ssh/vio-on-arm
```

### Issue: `Could not read from remote repository`

**Solutions:**
```bash
# Verify SSH config
cat ~/.ssh/config
# Should have entry like:
# Host github.com-vio-on-arm
#     HostName github.com
#     User git
#     IdentityFile ~/.ssh/vio-on-arm

# Test connection
ssh -vT git@github.com-vio-on-arm

# Clone with correct URL
git clone git@github.com-vio-on-arm:ZHosein/vio-on-arm.git
```

### Issue: SSH works for git but not Docker

**Expected Behavior:**
SSH deploy keys ONLY work for git operations, not for Docker/packages.

**Solution:**
You need a token (GitHub App or PAT) for Docker image access:
```bash
# Generate token
./scripts/generate-app-token.sh APP_ID INSTALL_ID key.pem

# Use for Docker
echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin
docker pull ghcr.io/zhosein/vio-on-arm:latest
```

---

## General Authentication Issues

### Issue: Not sure which authentication method to use

**Decision Tree:**

Need Docker images? → Yes → Use GitHub App or PAT (NOT SSH)
                   → No  → SSH Deploy Key is simplest

Multiple users? → Yes → GitHub App (better audit/control)
               → No  → Personal Access Token (simpler)

### Issue: Token expired

**Solutions:**

For GitHub App tokens (expire after 1 hour):
```bash
# Simply regenerate
./scripts/generate-app-token.sh APP_ID INSTALL_ID key.pem
```

For Personal Access Tokens:
```bash
# Check expiration: GitHub Settings → Developer settings → Personal access tokens
# If expired, create a new one with same permissions
# Update your commands with new token
```

### Issue: Don't have required credentials

**For Users:**
Contact the repository owner and request:
- For GitHub App: App ID, Installation ID, Private Key
- For PAT: A fine-grained personal access token
- For SSH: Have them add your public SSH key as deploy key

**For Repository Owners:**
See [ACCESS_GRANT_TEMPLATE.md](ACCESS_GRANT_TEMPLATE.md) for how to share credentials.

### Issue: Need to revoke access

**For Repository Owners:**

Revoke GitHub App access:
```bash
# Settings → Developer settings → GitHub Apps → [Your App] → Install App
# Click Configure → Uninstall
# Or rotate the private key to invalidate old one
```

Revoke Personal Access Token:
```bash
# Settings → Developer settings → Personal access tokens
# Find token → Delete
```

Revoke SSH Deploy Key:
```bash
# Repository Settings → Deploy keys
# Find key → Delete
```

---

## Still Having Issues?

### Diagnostic Commands

```bash
# Check git configuration
git config --list | grep credential

# Check Docker login status
docker system info | grep -A 5 Registry

# Check SSH configuration
ssh -vT git@github.com

# Check file permissions
ls -la ~/.ssh/
ls -la ~/.docker/config.json

# Test network connectivity
ping github.com
curl -I https://ghcr.io
```

### Getting Help

1. **Review documentation:**
   - [Quick Start Guide](QUICK_START.md)
   - [GitHub App Setup](GITHUB_APP_SETUP.md)
   - [Authentication Flows](AUTHENTICATION_FLOWS.md)

2. **Check GitHub Status:**
   - https://www.githubstatus.com/

3. **Enable verbose output:**
   ```bash
   # Git
   GIT_CURL_VERBOSE=1 GIT_TRACE=1 git clone ...
   
   # Docker
   docker --debug pull ...
   ```

4. **Contact repository owner** if you need:
   - Access credentials
   - Permission changes
   - Verification that you have correct access

---

## Security Reminders

- Never commit tokens or private keys to git
- Always use HTTPS with tokens (not plain passwords)
- Regenerate tokens if accidentally exposed
- Use environment variables for tokens:
  ```bash
  export GITHUB_TOKEN=your_token
  git clone https://x-access-token:$GITHUB_TOKEN@github.com/...
  ```
- Store private keys with restricted permissions:
  ```bash
  chmod 600 private-key.pem
  ```