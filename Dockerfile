FROM gcc:13 AS builder
WORKDIR /app
COPY . .
RUN g++ -std=c++17 -O2 -pthread main.cpp -o campus_nav

FROM debian:bookworm-slim
WORKDIR /app
RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/campus_nav /app/campus_nav
RUN chmod +x /app/campus_nav
EXPOSE 8080
CMD ["./campus_nav"]