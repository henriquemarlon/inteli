START_LOG = @echo "================================================= START OF LOG ==================================================="
END_LOG = @echo "================================================== END OF LOG ===================================================="

.PHONY: env
env:
	@cp ./.env.tmpl ./.env
	
.PHONY: run
run:
	$(START_LOG)
	@docker compose -f \
		./compose.yml up --build -d
	$(END_LOG)

.PHONY: database
database:
	@docker compose -f ./compose.yml up postgres --build