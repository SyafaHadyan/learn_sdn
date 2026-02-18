# Dockerfile for Mininet

# ==============================================================================
# Stage 1: Builder - Mininet compilation and dependency preparation
# ==============================================================================
FROM debian:bookworm AS builder

ARG MININET_VERSION=master
ARG OS_KEN_VERSION=2.11.2
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies (will not be in final image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    ca-certificates \
    python3-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Clone Mininet source (shallow clone for smaller size)
RUN git clone --depth 1 --branch ${MININET_VERSION} \
    https://github.com/mininet/mininet.git /opt/mininet

# Clone os-ken (Ryu fork) controller source (optional, can be removed if not needed)
RUN git clone --depth 1 --branch ${OS_KEN_VERSION} \
    https://github.com/openstack/os-ken.git /opt/os-ken

# ==============================================================================
# Stage 2: Production Runtime
# ==============================================================================
FROM debian:bookworm

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV MININET_DOCKER=true

# Metadata labels for image identification and compliance
LABEL org.opencontainers.image.title="Learn SDN" \
      org.opencontainers.image.description="Learn SDN with Mininet" \
      org.opencontainers.image.version="1.0.0" \
      org.opencontainers.image.source="https://github.com/mininet/mininet" \
      org.opencontainers.image.licenses="MIT" 

# Prevent services from being started during image build (temporary)
RUN printf '%s\n' '#!/bin/sh' 'exit 101' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

# Install runtime dependencies in a single layer for efficiency
# Organized by functional category for maintainability
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python runtime and utilities
    python3 \
    python3-pip \
    python-is-python3 \
    python3-termcolor \
    python3-setuptools \
    python3-mako \
    python3-bottle \
    python3-flask \
    # Core system utilities
    sudo \
    ca-certificates \
    lsb-release \
    patch \
    # Networking packages
    openvswitch-switch \
    openvswitch-testcontroller \
    iproute2 \
    iputils-ping \
    arping \
    iperf3 \
    tshark \
    traceroute \
    iptables \
    iptables-persistent \
    net-tools \
    bridge-utils \
    tcpdump \
    socat \
    dnsutils \
    curl \
    procps \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Configure pip for system packages (required for Mininet)
RUN pip3 config set global.break-system-packages true

# Copy pre-built Mininet from builder stage (avoids recompilation)
COPY --from=builder /opt/mininet /opt/mininet
# Copy pre-built os-ken (Ryu fork) controller from builder stage (optional)
COPY --from=builder /opt/os-ken /opt/os-ken

# Install Mininet with production-optimized flags
# Uses non-interactive installation and cleans up source after
RUN cd /opt/mininet && \
    apt-get update && \
    if ! apt-cache show openvswitch-controller >/dev/null 2>&1; then \
      apt-get install -y --no-install-recommends openvswitch-testcontroller || true; \
    fi && \
    PYTHON=python3 ./util/install.sh -nfv && \
    # Clean up build artifacts to reduce image size
    rm -rf /opt/mininet/.git \
           /opt/mininet/examples \
           /opt/mininet/doc \
           /var/lib/apt/lists/* \
           /tmp/* \
           /var/tmp/*

# Install os-ken (Ryu fork) controller (optional, can be removed if not needed)
RUN cd /opt/os-ken && \
    pip3 install -r requirements.txt && \
    python3 setup.py install && \
    rm -rf /tmp/* /var/tmp/*

# Remove build-time policy to allow services to start normally at runtime
RUN rm -f /usr/sbin/policy-rc.d || true

# Create application-specific directories
RUN mkdir -p /learn_sdn \
    && chmod 755 /learn_sdn

# Copy and set up entrypoint (keeping your working version)
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Switch to non-root user
USER root

WORKDIR /learn_sdn

# Health check to verify service readiness
HEALTHCHECK --interval=30s --timeout=10s --start-period=15s --retries=3 \
    CMD ovs-vsctl show > /dev/null 2>&1 || exit 1

# Expose Controller port (default OpenFlow port)
EXPOSE 6633/tcp
EXPOSE 6653/tcp

# Service start script (handles initialization)
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]