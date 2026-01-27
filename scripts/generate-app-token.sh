#!/bin/bash

# GitHub App Installation Token Generator
# This script generates an installation access token for a GitHub App
# Usage: ./generate-app-token.sh <app_id> <installation_id> <private_key_path>

set -e

# Check if required commands are available
command -v openssl >/dev/null 2>&1 || { echo "Error: openssl is required but not installed." >&2; exit 1; }
command -v curl >/dev/null 2>&1 || { echo "Error: curl is required but not installed." >&2; exit 1; }

# Check arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <app_id> <installation_id> <private_key_path>"
    echo ""
    echo "Arguments:"
    echo "  app_id           - Your GitHub App ID"
    echo "  installation_id  - The installation ID for your GitHub App"
    echo "  private_key_path - Path to your GitHub App's private key (.pem file)"
    echo ""
    echo "Example:"
    echo "  $0 123456 789012 /path/to/private-key.pem"
    exit 1
fi

APP_ID="$1"
INSTALLATION_ID="$2"
PRIVATE_KEY_PATH="$3"

# Check if private key file exists
if [ ! -f "$PRIVATE_KEY_PATH" ]; then
    echo "Error: Private key file not found at: $PRIVATE_KEY_PATH"
    exit 1
fi

# Generate JWT
echo "Generating JWT..."

# Current time
NOW=$(date +%s)
# Expiration time (10 minutes from now, GitHub allows max 10 minutes)
EXP=$((NOW + 600))

# Create JWT header
HEADER=$(echo -n '{"alg":"RS256","typ":"JWT"}' | openssl base64 -e -A | tr '+/' '-_' | tr -d '=')

# Create JWT payload
PAYLOAD=$(echo -n "{\"iat\":${NOW},\"exp\":${EXP},\"iss\":\"${APP_ID}\"}" | openssl base64 -e -A | tr '+/' '-_' | tr -d '=')

# Create signature
SIGNATURE=$(echo -n "${HEADER}.${PAYLOAD}" | openssl dgst -sha256 -sign "$PRIVATE_KEY_PATH" | openssl base64 -e -A | tr '+/' '-_' | tr -d '=')

# Combine to create JWT
JWT="${HEADER}.${PAYLOAD}.${SIGNATURE}"

echo "JWT generated successfully."
echo ""

# Get installation access token
echo "Requesting installation access token..."
RESPONSE=$(curl -s -X POST \
  -H "Authorization: Bearer ${JWT}" \
  -H "Accept: application/vnd.github+json" \
  "https://api.github.com/app/installations/${INSTALLATION_ID}/access_tokens")

# Extract token from response
TOKEN=$(echo "$RESPONSE" | grep -o '"token":[[:space:]]*"[^"]*' | cut -d'"' -f4)

if [ -z "$TOKEN" ]; then
    echo "Error: Failed to generate token."
    echo "Response from GitHub:"
    echo "$RESPONSE"
    exit 1
fi

echo "Installation access token generated successfully!"
echo ""
echo "=========================================="
echo "YOUR ACCESS TOKEN:"
echo "$TOKEN"
echo "=========================================="
echo ""
echo "This token is valid for 1 hour."
echo ""
echo "Usage examples:"
echo ""
echo "1. Clone the repository:"
echo "   git clone https://x-access-token:${TOKEN}@github.com/ZHosein/vio-on-arm.git"
echo ""
echo "2. Login to GitHub Container Registry:"
echo "   echo ${TOKEN} | docker login ghcr.io -u x-access-token --password-stdin"
echo ""
echo "3. Pull Docker image:"
echo "   docker pull ghcr.io/zhosein/vio-on-arm:latest"
echo ""
