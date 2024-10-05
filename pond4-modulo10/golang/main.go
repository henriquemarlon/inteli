package main

import (
	_ "github.com/henriquemarlon/pond4-modulo10/api"
	"log"
	"os"
	"github.com/henriquemarlon/pond4-modulo10/internal/infra/web/handler"
	"github.com/gin-gonic/gin"
	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
	"github.com/henriquemarlon/pond4-modulo10/internal/infra/database"
	swaggerFiles "github.com/swaggo/files"
	ginSwagger "github.com/swaggo/gin-swagger"
	"gorm.io/driver/sqlite"
	"gorm.io/gorm"
)

//	@title			Manager API
//	@version		1.0
//	@description	This is a.
//	@termsOfService	http://swagger.io/terms/

//	@contact.name	Manager API Support
//	@contact.url	https://github.com/henriquemarlon
//	@contact.email	email@example.com

//	@license.name	Apache 2.0
//	@license.url	http://www.apache.org/licenses/LICENSE-2.0.html

//	@host	localhost
//	@BasePath	/service2

func main() {
	db, err := gorm.Open(sqlite.Open("data/database.db"), &gorm.Config{})
	if err != nil {
		log.Fatal("failed to connect database")
	}

	db.AutoMigrate(&entity.User{})

	f, err := os.OpenFile("/logs/service2.log", os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		log.Fatal(err)
	}
	gin.DefaultWriter = f

	//////////////////////// Depensency Injection ///////////////////////////
	userRespository := database.NewUserRepositoryGorm(db)
	userHandlers := handler.NewUserHandlers(userRespository)

	r := gin.Default()

	router := r.Group("/service2")

	r.Use(gin.LoggerWithWriter(f))

	api := router.Group("/user")
	{
		api.GET("", userHandlers.FindAllUsers)
		api.POST("", userHandlers.CreateUser)
		api.PUT("/:id", userHandlers.UpdateUser)
		api.GET("/:id", userHandlers.FindUserById)
		api.DELETE("/:id", userHandlers.DeleteUser)
	}

	router.GET("/docs/*any", ginSwagger.WrapHandler(swaggerFiles.Handler))

	r.Run(":8080")
}