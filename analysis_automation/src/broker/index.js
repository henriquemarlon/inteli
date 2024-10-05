const aedes = require('aedes')()
const server = require('net').createServer(aedes.handle)

const port = 1883 // Porta padrão do MQTT

server.listen(port, "0.0.0.0", function () {
  console.log(`Servidor MQTT Aedes rodando na porta ${port}`)
})

aedes.on('subscribe', function (subscriptions, client) {
  console.log(`Cliente ${client.id} se inscreveu em tópicos: ${subscriptions.map(s => s.topic).join(',')}`)
})

aedes.on('unsubscribe', function (subscriptions, client) {
  console.log(`Cliente ${client.id} cancelou inscrição em tópicos: ${subscriptions.map(s => s.topic).join(',')}`)
})

aedes.on('client', function (client) {
  console.log(`Cliente ${client.id} conectado`)
})

aedes.on('clientDisconnect', function (client) {
  console.log(`Cliente ${client.id} desconectado`)
})

aedes.on('publish', function (packet, client) {
  console.log(`Cliente ${client ? client.id : 'n/a'} publicou mensagem no tópico ${packet.topic}: ${packet.payload.toString()}`)
})