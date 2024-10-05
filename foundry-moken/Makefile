# LOADING ENV FILE
-include .env

.PHONY: moken property

# DEFAULT VARIABLES	
START_LOG = @echo "==================== START OF LOG ===================="
END_LOG = @echo "==================== END OF LOG ======================"

define deploy_moken
	$(START_LOG)
	@forge script script/DeployMoken.s.sol --rpc-url $(RPC_URL) --broadcast --legacy
	$(END_LOG)
endef

define deploy_property
	$(START_LOG)
	@forge script script/DeployProperty.s.sol --rpc-url $(RPC_URL) --broadcast --legacy
	$(END_LOG)
endef

env: .env.tmpl
	cp .env.tmpl .env

moken:
	@echo "Deploying moken..."
	@$(deploy_moken)

property:
	@echo "Deploying properties..."
	@$(deploy_property)