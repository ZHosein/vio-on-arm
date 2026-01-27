# Authentication Flow Diagrams

This document provides visual representations of the authentication flows for accessing the vio-on-arm repository and Docker images.

## Overview of Access Methods

```
┌─────────────────────────────────────────────────────────┐
│         Access Methods for vio-on-arm                   │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────────┐  ┌──────────────────┐            │
│  │   Repository     │  │  Docker Images   │            │
│  │   (Git Clone)    │  │   (GHCR Pull)    │            │
│  └────────┬─────────┘  └────────┬─────────┘            │
│           │                     │                       │
│           ▼                     ▼                       │
│  ┌────────────────────────────────────────┐            │
│  │        Authentication Required          │            │
│  └────────────────┬───────────────────────┘            │
│                   │                                     │
│      ┌────────────┼────────────┐                       │
│      ▼            ▼             ▼                       │
│  ┌────────┐  ┌────────┐  ┌──────────┐                 │
│  │GitHub  │  │Personal│  │   SSH    │                 │
│  │  App   │  │ Access │  │  Deploy  │                 │
│  │        │  │ Token  │  │   Key    │                 │
│  │(Repo+  │  │(Repo+  │  │(Repo     │                 │
│  │Package)│  │Package)│  │ Only)    │                 │
│  └────────┘  └────────┘  └──────────┘                 │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## Method 1: GitHub App Flow

### For Repository Owner

```
┌─────────────────────────────────────────────────────────┐
│              Repository Owner Setup                      │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │ 1. Create GitHub App            │
        │    - Set permissions            │
        │    - Generate private key       │
        └────────────┬────────────────────┘
                     │
                     ▼
        ┌─────────────────────────────────┐
        │ 2. Install App on Repository    │
        │    - Get Installation ID        │
        └────────────┬────────────────────┘
                     │
                     ▼
        ┌─────────────────────────────────┐
        │ 3. Share Credentials (Secure)   │
        │    - App ID                     │
        │    - Installation ID             │
        │    - Private Key (.pem)         │
        └─────────────────────────────────┘
```

### For User

```
┌─────────────────────────────────────────────────────────┐
│              User Access Flow                            │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────┐
        │ 1. Receive Credentials          │
        │    - App ID                     │
        │    - Installation ID             │
        │    - Private Key                │
        └────────────┬────────────────────┘
                     │
                     ▼
        ┌─────────────────────────────────┐
        │ 2. Generate JWT Token           │
        │    (Using private key)          │
        └────────────┬────────────────────┘
                     │
                     ▼
        ┌─────────────────────────────────┐
        │ 3. Request Installation Token   │
        │    (JWT → GitHub API)           │
        └────────────┬────────────────────┘
                     │
                     ▼
        ┌─────────────────────────────────┐
        │ 4. Receive Installation Token   │
        │    (Valid for 1 hour)           │
        └────────────┬────────────────────┘
                     │
         ┌───────────┴────────────┐
         ▼                        ▼
┌─────────────────┐    ┌──────────────────┐
│ 5a. Clone Repo  │    │ 5b. Pull Image   │
│     (Git)       │    │     (Docker)     │
└─────────────────┘    └──────────────────┘
```

### Script Workflow

```
┌──────────────────────────────────────────────────────┐
│           generate-app-token.sh/py                    │
└──────────────────────────────────────────────────────┘
                       │
         ┌─────────────┴─────────────┐
         ▼                           ▼
┌─────────────────┐         ┌──────────────────┐
│   Read Inputs   │         │  Read Private    │
│   - App ID      │         │      Key         │
│   - Install ID  │         │    (PEM File)    │
└────────┬────────┘         └────────┬─────────┘
         │                           │
         └──────────┬────────────────┘
                    ▼
         ┌──────────────────────┐
         │   Create JWT         │
         │   - Header           │
         │   - Payload          │
         │   - Sign with key    │
         └──────────┬───────────┘
                    ▼
         ┌──────────────────────┐
         │   Call GitHub API    │
         │   POST /access_token │
         └──────────┬───────────┘
                    ▼
         ┌──────────────────────┐
         │  Return Token        │
         │  (Valid 1 hour)      │
         └──────────────────────┘
```

## Method 2: Personal Access Token Flow

```
┌─────────────────────────────────────────────────────────┐
│              Personal Access Token                       │
└─────────────────────────────────────────────────────────┘

Repository Owner:                    User:
┌──────────────────┐                ┌──────────────────┐
│ 1. Create Token  │                │                  │
│    in GitHub     │                │                  │
│    Settings      │                │                  │
└────────┬─────────┘                │                  │
         │                          │                  │
         ▼                          │                  │
┌──────────────────┐                │                  │
│ 2. Set Perms:    │                │                  │
│    - Contents    │                │                  │
│    - Packages    │                │                  │
└────────┬─────────┘                │                  │
         │                          │                  │
         ▼                          │                  │
┌──────────────────┐     Share      │                  │
│ 3. Copy Token    ├───(Secure)────►│ 4. Receive Token │
└──────────────────┘                └────────┬─────────┘
                                             │
                                             ▼
                              ┌──────────────────────────┐
                              │ 5. Use Token Directly    │
                              │    - Git clone           │
                              │    - Docker pull         │
                              └──────────────────────────┘
```

## Method 3: SSH Deploy Key Flow

```
┌─────────────────────────────────────────────────────────┐
│              SSH Deploy Key (Git Only)                   │
└─────────────────────────────────────────────────────────┘

User:                            Repository Owner:
┌──────────────────┐             ┌──────────────────┐
│ 1. Generate SSH  │             │                  │
│    Key Pair      │             │                  │
└────────┬─────────┘             │                  │
         │                       │                  │
         ▼                       │                  │
┌──────────────────┐             │                  │
│ 2. Share Public  ├───────────► │ 3. Add to Deploy │
│    Key           │   (Email)   │    Keys          │
└──────────────────┘             └────────┬─────────┘
                                          │
┌──────────────────┐                     │
│ 4. Configure     │◄────(Notify)────────┘
│    SSH           │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ 5. Clone Repo    │
│    (SSH)         │
└──────────────────┘

Note: Deploy Keys do NOT work for Docker images!
```

## Token Expiration and Renewal

```
┌─────────────────────────────────────────────────────────┐
│              Token Lifecycle                             │
└─────────────────────────────────────────────────────────┘

GitHub App Token:
┌────────────┐    1 hour    ┌─────────┐    Regenerate   ┌────────────┐
│  Generate  ├─────────────►│ Expires ├────────────────►│  Generate  │
│   Token    │              │         │  (Auto/Manual)  │    New     │
└────────────┘              └─────────┘                 └────────────┘

Personal Access Token:
┌────────────┐   Days/Months  ┌─────────┐   Create New   ┌────────────┐
│   Create   ├───────────────►│ Expires ├───────────────►│   Create   │
│   Token    │  (User-set)    │         │  (Manual)      │    New     │
└────────────┘                └─────────┘                └────────────┘

SSH Deploy Key:
┌────────────┐                ┌─────────────┐
│    Add     ├───────────────►│  No Expiry  │
│    Key     │  Indefinite    │  (Unless    │
└────────────┘                │  Removed)   │
                              └─────────────┘
```

## Security Comparison

```
┌──────────────────────────────────────────────────────────────────┐
│                 Security Features Comparison                      │
├─────────────┬──────────┬────────────┬─────────┬──────────────────┤
│   Method    │   Repo   │  Packages  │  Audit  │  Granularity     │
├─────────────┼──────────┼────────────┼─────────┼──────────────────┤
│ GitHub App  │    ✓     │     ✓      │    ✓    │  Per-app         │
│             │          │            │         │  (best)          │
├─────────────┼──────────┼────────────┼─────────┼──────────────────┤
│ PAT         │    ✓     │     ✓      │    ✓    │  Per-token       │
│ (Fine)      │          │            │         │  (good)          │
├─────────────┼──────────┼────────────┼─────────┼──────────────────┤
│ Deploy Key  │    ✓     │     ✗      │    ✗    │  Per-key         │
│ (SSH)       │          │            │         │  (limited)       │
└─────────────┴──────────┴────────────┴─────────┴──────────────────┘
```

## Recommended Choice Based on Use Case

```
┌─────────────────────────────────────────────────────────────────┐
│                    Use Case Guide                                │
└─────────────────────────────────────────────────────────────────┘

┌────────────────────────┐
│  Need Git + Docker?    │
└───────────┬────────────┘
            │
       ┌────┴────┐
       │   Yes   │
       └────┬────┘
            │
   ┌────────┴────────┐
   │  Multiple Users │
   └────────┬────────┘
            │
       ┌────┴────┐
       │   Yes   │──────► Use GitHub App (Best audit, per-user control)
       └─────────┘
            │
       ┌────┴────┐
       │    No   │──────► Use Personal Access Token (Simpler)
       └─────────┘

┌────────────────────────┐
│  Need only Git?        │
└───────────┬────────────┘
            │
       ┌────┴────┐
       │   Yes   │──────► Can use SSH Deploy Key (Simplest)
       └─────────┘
```

## Quick Reference Commands

### GitHub App
```bash
# Generate token
./scripts/generate-app-token.sh APP_ID INSTALL_ID key.pem

# Clone repo
git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git

# Docker login
echo TOKEN | docker login ghcr.io -u x-access-token --password-stdin
```

### Personal Access Token
```bash
# Clone repo
git clone https://x-access-token:TOKEN@github.com/ZHosein/vio-on-arm.git

# Docker login
echo TOKEN | docker login ghcr.io -u USERNAME --password-stdin
```

### SSH Deploy Key
```bash
# Clone repo
git clone git@github.com:ZHosein/vio-on-arm.git

# Note: Does NOT work for Docker images
```