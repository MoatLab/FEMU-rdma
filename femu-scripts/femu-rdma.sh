#!/usr/bin/env bash
set -e

echo "=== FEMU / RDMA environment setup ==="

# Ensure we are in bash
if [ -z "$BASH_VERSION" ]; then
  echo "[ERROR] Not running in bash. Run: bash"
  exit 1
fi

echo "[OK] Running in bash: $BASH_VERSION"

# 1. Fix PATH (RDMA tools live in /usr/sbin)
echo "[INFO] Fixing PATH..."
export PATH="$PATH:/usr/sbin"

if ! grep -q "/usr/sbin" ~/.bashrc; then
  echo 'export PATH=$PATH:/usr/sbin' >> ~/.bashrc
fi

# 2. Update system
echo "[INFO] Updating apt..."
sudo apt update

# 3. Install required packages
echo "[INFO] Installing build + RDMA dependencies..."
sudo apt install -y \
  pkg-config \
  libglib2.0-dev \
  libpixman-1-dev \
  rdma-core \
  libibverbs-dev \
  ibverbs-providers \
  infiniband-diags \
  build-essential \
  ninja-build \
  python3 \
  python3-pip


# 4. Verify pkg-config
echo "[CHECK] pkg-config"
which pkg-config
pkg-config --version
pkg-config --modversion glib-2.0

# 5. Verify RDMA userspace
echo "[CHECK] RDMA (ibverbs)"
if command -v ibv_devinfo >/dev/null 2>&1; then
  ibv_devinfo | head -n 20
else
  echo "[WARN] ibv_devinfo not in PATH, trying /usr/sbin"
  /usr/sbin/ibv_devinfo | head -n 20
fi

# 6. Shell reminder
echo
echo "=== DONE ==="

#FIX DNS
sudo resolvectl dns eno1 8.8.8.8 1.1.1.1
sudo resolvectl domain eno1 ~.
resolvectl status