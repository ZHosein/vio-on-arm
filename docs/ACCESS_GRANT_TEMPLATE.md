# Access Grant Template

This is a template for repository owners to use when granting access to users.

---

## Granting Access to Users

### Using GitHub App (Recommended)

**To grant access to a new user:**

1. Share the following information securely (NOT via email - use encrypted channels):

```
GitHub App Credentials for vio-on-arm Access
============================================

App ID: [YOUR_APP_ID]
Installation ID: [YOUR_INSTALLATION_ID]

Private Key: [Attach the .pem file separately via secure channel]

Instructions: See https://github.com/ZHosein/vio-on-arm/blob/main/docs/QUICK_START.md
```

2. Send the private key file separately through a secure channel (e.g., encrypted file sharing, password-protected archive)

3. Provide access instructions:
   - Users should save credentials securely
   - They can generate tokens using the provided scripts
   - Tokens expire after 1 hour and need to be regenerated

### Using Personal Access Token

**To grant access to a new user:**

1. Create a fine-grained personal access token:
   - Go to: Settings → Developer settings → Personal access tokens → Fine-grained tokens
   - Click "Generate new token"
   - Set repository access to `vio-on-arm` only
   - Grant permissions: Contents (Read), Packages (Read)
   - Set appropriate expiration date
   - Generate and copy token

2. Share securely with user:

```
Access Token for vio-on-arm
===========================

Token: [YOUR_TOKEN]
Expires: [EXPIRATION_DATE]

To clone:
git clone https://x-access-token:[TOKEN]@github.com/ZHosein/vio-on-arm.git

To pull Docker image:
echo [TOKEN] | docker login ghcr.io -u x-access-token --password-stdin
docker pull ghcr.io/zhosein/vio-on-arm:latest

Full instructions: https://github.com/ZHosein/vio-on-arm/blob/main/docs/QUICK_START.md
```

### Using SSH Deploy Key (Git Only)

**To grant access to a new user:**

1. Request user's public SSH key
2. Add it to repository:
   - Go to: Repository Settings → Deploy keys → Add deploy key
   - Title: "[User Name] - Read Only"
   - Key: [Paste user's public key]
   - Leave "Allow write access" unchecked
   - Click "Add key"

3. Notify user:

```
SSH Deploy Key Added
===================

Your SSH public key has been added to the repository.

To clone:
git clone git@github.com:ZHosein/vio-on-arm.git

Note: For Docker image access, you'll need a separate token.

Full instructions: https://github.com/ZHosein/vio-on-arm/blob/main/docs/QUICK_START.md
```

## Revoking Access

### GitHub App
- Uninstall the app from the repository, or
- Rotate the app's private key (generates new key, invalidates old one)

### Personal Access Token
- Delete the specific token in Settings → Developer settings → Personal access tokens

### SSH Deploy Key
- Go to Repository Settings → Deploy keys
- Click "Delete" next to the specific key

## Access Audit

### GitHub App
- View in Settings → Developer settings → GitHub Apps → [Your App] → Advanced → Recent Deliveries
- Check installation access logs

### Personal Access Token
- View token usage in Settings → Developer settings → Personal access tokens
- Check "Last used" column

### SSH Deploy Key
- No built-in audit logs
- Check git commit/pull logs in repository insights

## Best Practices

1. **Use separate credentials per user** - Makes revocation easier
2. **Set expiration dates** - Tokens should expire after reasonable periods
3. **Document who has access** - Keep a record (not in the repository!)
4. **Rotate credentials periodically** - Update keys/tokens every 3-6 months
5. **Use least privilege** - Only grant read access unless write is needed
6. **Secure credential sharing** - Use encrypted channels, never plain email

## Credential Storage Template (Keep this private!)

```
User Access Log (DO NOT COMMIT TO REPO)
========================================

User: [Name/Email]
Date Granted: [Date]
Access Method: [GitHub App / PAT / Deploy Key]
Token/Key ID: [Identifier]
Expiration: [Date]
Revoked: [Yes/No - Date if yes]
Notes: [Any relevant notes]

---

User: [Next user]
...
```

Save this as a private file (e.g., `ACCESS_LOG.private.txt`) and add to `.gitignore`.