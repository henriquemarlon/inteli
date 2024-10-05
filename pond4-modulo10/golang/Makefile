.PHONY: swagger
swagger:
	$(START_LOG)
	@if command -v swag >/dev/null 2>&1; then \
		swag init -g ./main.go -o ./api; \
	else \
		docker run --rm -v $$(pwd):/code ghcr.io/swaggo/swag:latest init -g ./main.go -o ./api; \
	fi
	@go mod tidy
	$(END_LOG)