-include .env

.PHONY: env
env:
	@cp ./.env.tmpl ./.env

.PHONY: test
test:
	@go test ./... -coverprofile=./test/coverage_sheet.md -v

.PHONY: coverage
coverage: test
	@go tool cover -html=./test/coverage_sheet.md

.PHONY: run
run:
	@docker compose -f \
		./compose.yaml --env-file ./.env up --build