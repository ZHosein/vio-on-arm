# GitHub App Setup Guide

This guide explains how to set up and use the GitHub App for accessing this repository and its Docker images from GitHub Packages.

## Overview

Similar to how GitHub deploy keys work for single repositories, this GitHub App provides controlled access to:
- Clone this repository
- Pull Docker images from GitHub Container Registry

## For Repository Owners

### Creating the GitHub App

1. **Navigate to GitHub App creation page:**
   - Go to your GitHub account settings
   - Navigate to "Developer settings" > "GitHub Apps"
   - Click "New GitHub App"

2. **Use the manifest (Recommended):**
   - You can use the manifest file at `.github/github-app-manifest.json`
   - Or create manually with these settings:

3. **Configure the app with these settings:**
   - **GitHub App name:** `vio-on-arm-access` (or your preferred name)
   - **Homepage URL:** `https://github.com/ZHosein/vio-on-arm`
   - **Webhook:** Disable (uncheck "Active")
   - **Permissions:**
     - Repository permissions:
       - Contents: Read-only
       - Metadata: Read-only
     - Organization permissions:
       - Packages: Read-only
   - **Where can this GitHub App be installed:** Only on this account

4. **After creation:**
   - Note down the App ID
   - Generate and download a private key (.pem file)
   - Keep these credentials secure

5. **Install the app:**
   - Go to the app's settings
   - Click "Install App"
   - Select your account
   - Choose "Only select repositories" and select `vio-on-arm`
   - Click "Install"

## For Users

### Prerequisites

- Git installed on your system
- Docker installed on your system (for pulling images)
- Access granted by the repository owner (they'll share credentials with you)

### Option 1: Using Personal Access Token (Simplest)

The repository owner can create a fine-grained personal access token and share it with you:

1. **For Repository Access (Cloning):**
   ```bash
   # Clone using the token
   git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git
   ```

2. **For Docker Image Access:**
   ```bash
   # Login to GitHub Container Registry
   echo TOKEN | docker login ghcr.io -u USERNAME --password-stdin
   
   # Pull the image
   docker pull ghcr.io/zhosein/vio-on-arm:latest
   ```

### Option 2: Using GitHub App Installation Token (Advanced)

If the repository owner has set up a GitHub App and provided you with credentials:

1. **Get the credentials from repository owner:**
   - App ID
   - Installation ID
   - Private key file (.pem)

2. **Generate an installation access token:**
   
   Use the provided script (see `scripts/generate-app-token.sh`) or:
   
   ```bash
   # Run the token generation script
   ./scripts/generate-app-token.sh APP_ID INSTALLATION_ID private-key.pem
   ```

3. **Use the token for git operations:**
   ```bash
   # Clone the repository
   git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git
   ```

4. **Use the token for Docker:**
   ```bash
   # Login to GitHub Container Registry
   echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin
   
   # Pull the image
   docker pull ghcr.io/zhosein/vio-on-arm:latest
   ```

### Option 3: Using SSH Deploy Keys (Git Only)

For read-only git access, you can use SSH deploy keys:

1. **Generate an SSH key pair:**
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/vio-on-arm-deploy-key
   ```

2. **Request the repository owner to add your public key:**
   - Share your public key (`~/.ssh/vio-on-arm-deploy-key.pub`) with the repository owner
   - They will add it as a deploy key in repository settings

3. **Configure SSH:**
   ```bash
   # Add to ~/.ssh/config
   Host github.com-vio-on-arm
       HostName github.com
       User git
       IdentityFile ~/.ssh/vio-on-arm-deploy-key
       IdentitiesOnly yes
   ```

4. **Clone using SSH:**
   ```bash
   git clone git@github.com-vio-on-arm:ZHosein/vio-on-arm.git
   ```

   Note: SSH deploy keys only work for git operations, not for Docker image access.

## Security Best Practices

1. **For Repository Owners:**
   - Use fine-grained permissions (read-only)
   - Rotate tokens/keys periodically
   - Monitor access logs
   - Use different tokens for different users
   - Set expiration dates on tokens when possible

2. **For Users:**
   - Never share your credentials
   - Store credentials securely (use environment variables or credential managers)
   - Don't commit credentials to repositories
   - Use tokens with minimum required permissions

## Troubleshooting

### "authentication required" error when pulling Docker image

Make sure you're logged in to GitHub Container Registry:
```bash
docker login ghcr.io
```

### "repository not found" error when cloning

- Verify you have the correct access token
- Check that the token hasn't expired
- Ensure the token has `contents: read` permission

### Token has expired

- Personal access tokens can be set to expire
- GitHub App installation tokens expire after 1 hour
- Generate a new token following the steps above

## Additional Resources

- [GitHub Apps Documentation](https://docs.github.com/en/apps)
- [Personal Access Tokens](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token)
- [Deploy Keys](https://docs.github.com/en/developers/overview/managing-deploy-keys)
- [GitHub Container Registry Authentication](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)
