# Base image with Python and minimal X11 support
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Webots dependencies
RUN apt-get update && apt-get install -y \
    wget \
    python3 \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libx11-xcb1 \
    libxrender1 \
    libxrandr2 \
    libxi6 \
    libxtst6 \
    libxcomposite1 \
    libxcursor1 \
    libxdamage1 \
    libxt6 \
    unzip \
    x11-apps \
    && apt-get clean

# Install Webots
RUN wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb && \
    dpkg -i webots_2023b_amd64.deb || apt-get install -f -y

ENV WEBOTS_HOME=/usr/local/webots
ENV PATH="$WEBOTS_HOME:$PATH"

# Set working directory
WORKDIR /app
COPY . /app

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install -r requirements.txt

# Default command to run Webots GUI
CMD ["webots"]
