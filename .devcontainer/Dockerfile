# Multi-stage build for optimized image size
FROM mcr.microsoft.com/devcontainers/base:bookworm AS builder

# Build arguments for flexibility
ARG MININET_VERSION=master
ARG OS_KEN_VERSION=2.11.2
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    ca-certificates \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone Mininet source
RUN git clone --depth 1 --branch ${MININET_VERSION} \
    https://github.com/mininet/mininet.git /opt/mininet

# Clone os-ken (Ryu fork) controller source (optional, can be removed if not needed)
RUN git clone --depth 1 --branch ${OS_KEN_VERSION} \
    https://github.com/openstack/os-ken.git /opt/os-ken

# Production stage
FROM mcr.microsoft.com/devcontainers/base:bookworm
ENV DEBIAN_FRONTEND=noninteractive

# Prevent services from being started during image build (temporary)
RUN printf '%s\n' '#!/bin/sh' 'exit 101' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

# Metadata labels
LABEL org.opencontainers.image.title="Mininet DevContainer" \
      org.opencontainers.image.description="Mininet DevContainer with Linux Debian" \
      org.opencontainers.image.source="https://github.com/mininet/mininet" \
      org.opencontainers.image.licenses="MIT"

# Install runtime dependencies in optimized layers with BuildKit cache
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    # Core system packages
    python3 \
    python3-pip \
    python-is-python3 \
    python3-termcolor \
    python3-setuptools \
    python3-mako \
    python3-bottle \
    python3-flask \
    patch \
    sudo \
    lsb-release \
    ca-certificates \
    # Networking packages
    openvswitch-switch \
    openvswitch-testcontroller \
    iproute2 \
    iputils-ping \
    arping \
    net-tools \
    bridge-utils \
    tcpdump \
    traceroute \
    iptables \
    iptables-persistent \
    iperf3 \
    iperf \
    socat \
    telnet \
    netcat-traditional \
    curl \
    dnsutils \
    iputils-tracepath \
    tshark \
    vim \
    bc \
    git \
    procps \
    && rm -rf /tmp/* /var/tmp/*

# Copy Mininet source from builder stage
COPY --from=builder /opt/mininet /opt/mininet

# Copy os-ken (Ryu fork) controller source from builder stage (optional)
COPY --from=builder /opt/os-ken /opt/os-ken

# Install Mininet with your working configuration
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    cd /opt/mininet && \
    apt-get update && \
    # Configure pip to allow system packages (your working fix)
    pip3 config set global.break-system-packages true && \
    # If openvswitch-controller package is missing on this platform, install the testcontroller as a safe fallback
    if ! apt-cache show openvswitch-controller >/dev/null 2>&1; then apt-get install -y --no-install-recommends openvswitch-testcontroller || true; fi && \
    # Install with your working flags
    PYTHON=python3 ./util/install.sh -nfv && \
    # Clean up
    rm -rf /opt/mininet/.git /opt/mininet/examples /opt/mininet/doc


    
# Install os-ken (Ryu fork) controller (optional, can be removed if not needed)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    cd /opt/os-ken && \
    pip3 install -r requirements.txt && \
    python3 setup.py install && \
    rm -rf /tmp/* /var/tmp/*

# Remove build-time policy to allow services to start normally at runtime
RUN rm -f /usr/sbin/policy-rc.d || true

USER root
WORKDIR /workspaces/learn_sdn

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=15s --retries=3 \
    CMD ovs-vsctl show > /dev/null 2>&1 || exit 1

# Start OVS on container startup for devcontainer
RUN echo '#!/bin/bash\nservice openvswitch-switch start' > /usr/local/bin/start-ovs.sh && \
    chmod +x /usr/local/bin/start-ovs.sh

CMD ["sleep", "infinity"]
