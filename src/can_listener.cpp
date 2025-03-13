#include <ros/ros.h>
#include <std_msgs/String.h>
#include "icsneo/icsneocpp.h"
#include <vector>
#include <bitset>
#include <memory>

using namespace std;

// Classe para gerenciar a comunicação com o ValueCAN4-2
class CANReader {
public:
    CANReader() : device(nullptr) {}

    ~CANReader() { //Destrutor
        closeDevice();
    }

    bool initialize() {
        vector<shared_ptr<icsneo::Device>> devices = icsneo::FindAllDevices(); //Encontrar dispositivos
        if (devices.empty()) {
            ROS_ERROR_STREAM("Nenhum dispositivo CAN encontrado!");
            return false;
        }

        device = devices.front(); //PErmitir a interação com o dispositivo

        // Tenta abrir o dispositivo
        if (!device->open()) { //FUnção de open retorna se o dispositivo foi aberto com sucesso
            ROS_ERROR_STREAM("Falha ao abrir o dispositivo CAN: " << icsneo::GetLastError());
            return false;
        }

        ROS_INFO("Dispositivo conectado: %s", device->getProductName().c_str());

        // Coloca o dispositivo online para começar a receber mensagens
        if (!device->goOnline()) {
            device->close();
            ROS_ERROR("Falha ao colocar o dispositivo online!");
            return false;
        }

        // Habilita a coleta de mensagens no dispositivo
        device->enableMessagePolling();
        // device->setPollingMessageLimit(100000); // Caso queira limitar o número de mensagens recebidas

        return true;
    }

    string readMessages() {
        string dadosBinarios;
        vector<shared_ptr<icsneo::Message>> mensagens;
    
        if (device) {
            // Obtém apenas as mensagens novas; o buffer interno é reovado (só publicar as novas mensagens) -> Exige que o listener do node leia e salve em um buffer durante o processamento
            device->getMessages(mensagens);
    
            // Para cada mensagem recebida:
            for (const auto& msg : mensagens) {
                // Verifica se a mensagem é do tipo Frame
                if (msg->type == icsneo::Message::Type::Frame) {
                    // Faz o cast direto para CANMessage
                    auto canMsg = static_pointer_cast<icsneo::CANMessage>(msg);
                    // canmsg->arbid is valid here
                    // canmsg->data is an std::vector<uint8_t>, you can check .size() for the DLC of the message
                    // canmsg->timestamp is the time recorded by the hardware in nanoseconds since (1/1/2007 12:00:00 GMT)
                    
                    // Acrescenta os bytes da mensagem à string para publicar a mensagem sem processamento no ros
                    dadosBinarios.append(reinterpret_cast<const char*>(canMsg->data.data()),
                                         canMsg->data.size());
                }
            }
        }
        return dadosBinarios;
    }
    

    void closeDevice() {
        if (device) {
            device->close();
            ROS_INFO("Dispositivo fechado com sucesso.");
        }
    }

private:
    shared_ptr<icsneo::Device> device;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_listener");
    ros::NodeHandle nh;
    ros::Publisher can_pub = nh.advertise<std_msgs::String>("can_data", 1000);

    CANReader canReader;
    if (!canReader.initialize()) {
        return 1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) { //Loop de leitura e publicação das mensagens 
        string messages = canReader.readMessages();
        std_msgs::String ros_msg;
        ros_msg.data = messages;
        can_pub.publish(ros_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    canReader.closeDevice();
    return 0;
}
