# Quick Start Guide

This guide provides step-by-step instructions for both repository owners and users.

## For Repository Owners: Setting Up Access Control

### Step 1: Choose Your Access Control Method

You have three options to grant access to your repository and Docker images:

#### Option A: GitHub App (Recommended for Teams)
Best for: Multiple users, fine-grained control, audit logging

#### Option B: Personal Access Tokens
Best for: Quick setup, few users, simple use cases

#### Option C: Deploy Keys (Git Only)
Best for: Git access only, automated systems

### Step 2: Set Up GitHub App (Option A)

1. **Create the GitHub App:**
   - Go to GitHub Settings → Developer settings → GitHub Apps → New GitHub App
   - Use these settings:
     - **Name**: `vio-on-arm-access` (or your choice)
     - **Homepage URL**: `https://github.com/ZHosein/vio-on-arm`
     - **Webhook**: Uncheck "Active" (disable webhooks)
     - **Permissions**:
       - Repository: Contents (Read-only), Metadata (Read-only)
       - Packages: Read
     - **Where can this app be installed**: Only on this account

2. **After creation:**
   - Note the **App ID** (found in "About" section)
   - Generate and download a **private key** (`.pem` file)
   - Keep these secure!

3. **Install the app:**
   - Go to Install App → Select your account
   - Choose "Only select repositories"
   - Select `vio-on-arm`
   - Click Install
   - Note the **Installation ID** from the URL

4. **Share with users:**
   - App ID: `XXXX`
   - Installation ID: `YYYY`
   - Private key file (send securely, not via email/chat)

### Step 3: Create Personal Access Token (Option B)

1. Go to GitHub Settings → Developer settings → Personal access tokens → Fine-grained tokens
2. Click "Generate new token"
3. Configure:
   - **Repository access**: Only select repositories → Choose `vio-on-arm`
   - **Permissions**:
     - Repository: Contents (Read-only)
     - Packages: Read
   - **Expiration**: Set appropriate expiration
4. Generate and copy the token
5. Share with users (securely)

### Step 4: Add Deploy Key (Option C - Git Only)

1. Get the user's public SSH key
2. Go to repository Settings → Deploy keys → Add deploy key
3. Paste the public key
4. Leave "Allow write access" unchecked
5. Click "Add key"

Note: Deploy keys only work for git operations, not Docker images.

## For Users: Accessing the Repository and Images

### Prerequisites

Install required tools:
```bash
# Git
sudo apt-get install git  # Debian/Ubuntu
brew install git          # macOS

# Docker
# Follow instructions at https://docs.docker.com/get-docker/
```

### Method 1: Using GitHub App Token

**You need from repository owner:**
- App ID
- Installation ID  
- Private key file (`.pem`)

**Steps:**

1. **Save the private key:**
   ```bash
   # Save the .pem file to a secure location
   mkdir -p ~/.github-app
   mv github-app-key.pem ~/.github-app/
   chmod 600 ~/.github-app/github-app-key.pem
   ```

2. **Generate a token:**
   
   Using the bash script:
   ```bash
   cd vio-on-arm
   ./scripts/generate-app-token.sh APP_ID INSTALLATION_ID ~/.github-app/github-app-key.pem
   ```
   
   Or using Python:
   ```bash
   # Install dependencies first
   pip install PyJWT cryptography requests
   
   # Generate token
   python ./scripts/generate-app-token.py APP_ID INSTALLATION_ID ~/.github-app/github-app-key.pem
   ```

3. **Copy the token from the output** (it expires in 1 hour)

4. **Clone the repository:**
   ```bash
   git clone https://x-access-token:YOUR_TOKEN@github.com/ZHosein/vio-on-arm.git
   ```

5. **Pull Docker image:**
   ```bash
   # Login
   echo YOUR_TOKEN | docker login ghcr.io -u x-access-token --password-stdin
   
   # Pull
   docker pull ghcr.io/zhosein/vio-on-arm:latest
   ```

### Method 2: Using Personal Access Token

**You need from repository owner:**
- Personal access token
- Your GitHub username (for Docker login)

**Steps:**

1. **Clone the repository:**
   ```bash
   git clone https://x-access-token:YOUR_TOKEN@github.com/ZHosein/vio-on-arm.git
   ```

2. **Pull Docker image:**
   ```bash
   # Login
   echo YOUR_TOKEN | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin
   
   # Pull
   docker pull ghcr.io/zhosein/vio-on-arm:latest
   ```

### Method 3: Using SSH Deploy Key (Git Only)

**You need to provide repository owner:**
- Your public SSH key

**Steps:**

1. **Generate SSH key (if you don't have one):**
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/vio-on-arm
   ```

2. **Share your public key with repository owner:**
   ```bash
   cat ~/.ssh/vio-on-arm.pub
   ```

3. **Configure SSH:**
   ```bash
   # Add to ~/.ssh/config
   cat >> ~/.ssh/config << EOF
   Host github.com-vio-on-arm
       HostName github.com
       User git
       IdentityFile ~/.ssh/vio-on-arm
       IdentitiesOnly yes
   EOF
   ```

4. **Clone using SSH:**
   ```bash
   git clone git@github.com-vio-on-arm:ZHosein/vio-on-arm.git
   ```

Note: Deploy keys don't work for Docker images. You'll need a token for that.

## Using the Docker Image

Once you've pulled the image:

```bash
# Run interactively
docker run -it ghcr.io/zhosein/vio-on-arm:latest

# Run with dataset mounted
docker run -it -v /path/to/dataset:/root/dataset ghcr.io/zhosein/vio-on-arm:latest

# Run the SLAM application
docker run -it ghcr.io/zhosein/vio-on-arm:latest ./build/IMX -logs -calibration=/root/datasets/roomMap/calibration.json
```

## Troubleshooting

### "Authentication required" error

**For Git:**
- Verify your token hasn't expired
- Check token has `contents:read` permission
- Ensure correct token format in URL

**For Docker:**
- Run `docker logout ghcr.io` then login again
- Verify token hasn't expired
- Check token has `packages:read` permission

### "Repository not found" error

- Verify you're using the correct repository name
- Check your token has access to this specific repository
- Ensure the repository owner has granted you access

### Token expired (GitHub App tokens)

GitHub App installation tokens expire after 1 hour. Simply regenerate:
```bash
./scripts/generate-app-token.sh APP_ID INSTALLATION_ID ~/.github-app/github-app-key.pem
```

### Script permission denied

Make scripts executable:
```bash
chmod +x scripts/*.sh scripts/*.py
```

## Security Tips

- Never commit tokens or private keys to git
- Use environment variables for tokens:
  ```bash
  export GITHUB_TOKEN=your_token_here
  git clone https://x-access-token:$GITHUB_TOKEN@github.com/ZHosein/vio-on-arm.git
  ```
- Store private keys securely with restricted permissions (`chmod 600`)
- Rotate tokens/keys periodically
- Use separate tokens for different projects/users

## Need More Help?

- See [GitHub App Setup Guide](GITHUB_APP_SETUP.md) for detailed documentation
- See [Scripts README](../scripts/README.md) for script usage details
- Check [GitHub's documentation](https://docs.github.com/en/authentication) for authentication help
