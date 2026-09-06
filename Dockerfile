FROM gcc:13 AS builder
WORKDIR /app
COPY . .
RUN g++ -std=c++17 -O3 -pthread main.cpp -o campus_nav

FROM debian:bookworm-slim
WORKDIR /app
COPY --from=builder /app/campus_nav .
EXPOSE 8080
CMD ["./campus_nav"]