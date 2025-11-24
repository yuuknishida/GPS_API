# Use official lightweight Python image
FROM python:3.11-slim

# Metadata
LABEL org.opencontainers.image.source="https://github.com/your-repo/LoRaGPS_Project"

# Environment
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV GPS_DB_PATH=/data/gps_data.db

# Create app directory
WORKDIR /app

# Install system deps
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for build caching
COPY requirements.txt /app/requirements.txt

# Install python deps
RUN pip install --upgrade pip \
    && pip install --no-cache-dir -r /app/requirements.txt

# Copy application code
COPY . /app

# Ensure permissions
RUN mkdir -p /data && chown -R www-data:www-data /data /app

# Expose port
EXPOSE 5000

# Run using gunicorn
# Use 2 workers for smaller environments; change -w auto as needed
CMD ["gunicorn", "-w", "2", "-b", "0.0.0.0:5000", "app:app"]
