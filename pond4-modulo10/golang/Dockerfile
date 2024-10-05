FROM golang:1.22.4 AS build

WORKDIR /app

COPY . .

RUN go mod tidy

RUN CGO_ENABLED=1 GOOS=linux go build -o golang

FROM debian:latest AS final

WORKDIR /app

COPY --from=build /app/golang .

EXPOSE 8080

CMD ["./golang"]