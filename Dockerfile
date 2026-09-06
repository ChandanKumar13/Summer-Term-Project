FROM debian:bookworm-slim

WORKDIR /app

# Install native g++ and required libraries in the same environment
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY . .

# Compile directly in the target runtime environment
RUN g++ -std=c++17 -O2 -pthread main.cpp -o campus_nav

# Ensure binary is executable
RUN chmod +x campus_nav

# Expose Render standard ports
EXPOSE 10000 8080

CMD ["./campus_nav"]