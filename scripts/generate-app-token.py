#!/usr/bin/env python3
"""
GitHub App Installation Token Generator

This script generates an installation access token for a GitHub App.
Requires: PyJWT and cryptography libraries

Usage: python generate-app-token.py <app_id> <installation_id> <private_key_path>
"""

import sys
import time
import json
import argparse
from pathlib import Path

try:
    import jwt
    import requests
except ImportError:
    print("Error: Required libraries not installed.")
    print("Please install them with: pip install PyJWT cryptography requests")
    sys.exit(1)


def generate_jwt(app_id: str, private_key_path: str) -> str:
    """Generate a JWT for GitHub App authentication."""
    
    # Read the private key
    try:
        with open(private_key_path, 'r') as key_file:
            private_key = key_file.read()
    except FileNotFoundError:
        print(f"Error: Private key file not found at: {private_key_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading private key: {e}")
        sys.exit(1)
    
    # Current time
    now = int(time.time())
    
    # JWT payload
    payload = {
        'iat': now,           # Issued at time
        'exp': now + 600,     # JWT expiration time (10 minutes maximum)
        'iss': app_id         # GitHub App's identifier
    }
    
    # Generate JWT
    try:
        encoded_jwt = jwt.encode(payload, private_key, algorithm='RS256')
        return encoded_jwt
    except Exception as e:
        print(f"Error generating JWT: {e}")
        sys.exit(1)


def get_installation_token(jwt_token: str, installation_id: str) -> dict:
    """Get an installation access token using the JWT."""
    
    url = f"https://api.github.com/app/installations/{installation_id}/access_tokens"
    headers = {
        'Authorization': f'Bearer {jwt_token}',
        'Accept': 'application/vnd.github+json',
        'X-GitHub-Api-Version': '2022-11-28'
    }
    
    try:
        response = requests.post(url, headers=headers)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error requesting installation token: {e}")
        if hasattr(e.response, 'text'):
            print(f"Response: {e.response.text}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description='Generate GitHub App installation access token',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s 123456 789012 /path/to/private-key.pem
  %(prog)s 123456 789012 ./github-app-private-key.pem
        """
    )
    parser.add_argument('app_id', help='Your GitHub App ID')
    parser.add_argument('installation_id', help='The installation ID for your GitHub App')
    parser.add_argument('private_key_path', help="Path to your GitHub App's private key (.pem file)")
    
    args = parser.parse_args()
    
    print("Generating JWT...")
    jwt_token = generate_jwt(args.app_id, args.private_key_path)
    print("JWT generated successfully.\n")
    
    print("Requesting installation access token...")
    token_data = get_installation_token(jwt_token, args.installation_id)
    
    token = token_data.get('token')
    expires_at = token_data.get('expires_at')
    
    if not token:
        print("Error: No token in response")
        print(json.dumps(token_data, indent=2))
        sys.exit(1)
    
    print("Installation access token generated successfully!\n")
    print("=" * 50)
    print("YOUR ACCESS TOKEN:")
    print(token)
    print("=" * 50)
    print(f"\nExpires at: {expires_at}")
    print("\nUsage examples:\n")
    print("1. Clone the repository:")
    print(f"   git clone https://x-access-token:{token}@github.com/ZHosein/vio-on-arm.git\n")
    print("2. Login to GitHub Container Registry:")
    print(f"   echo {token} | docker login ghcr.io -u x-access-token --password-stdin\n")
    print("3. Pull Docker image:")
    print("   docker pull ghcr.io/zhosein/vio-on-arm:latest\n")
    print("4. Set as environment variable:")
    print(f"   export GITHUB_TOKEN={token}\n")


if __name__ == '__main__':
    main()