#include <ros/ros.h>
#include <std_msgs/String.h>
#include "icsneo/icsneocpp.h"
#include <vector>
#include <bitset>
#include <memory>

using namespace std;

class CANReader {
public:
    CANReader(ros::NodeHandle& nh) : device(nullptr) {
        // Configura o publisher no ROS
        can_pub = nh.advertise<std_msgs::String>("can_data", 1000);
    }

    ~CANReader() { //Detrutor da classe, por segurança
        closeDevice();
    }

    bool initialize() {
        //Encontra dispositivos
        vector<shared_ptr<icsneo::Device>> devices = icsneo::FindAllDevices();
        if (devices.empty()) {
            ROS_ERROR("Nenhum dispositivo CAN encontrado!");
            return false;
        }
        device = devices.front(); //Permite a interação com um dispositivo
        if (!device->open()) {
            ROS_ERROR("Falha ao abrir dispositivo CAN: %s", icsneo::GetLastError().c_str());
            return false;
        }

        ROS_INFO("Dispositivo conectado: %s", device->getGenericProductName().c_str());
        if (!device->goOnline()) { //Coloca o dispositivo online para começar a receber mensagens
            device->close();
            ROS_ERROR("Falha ao colocar o dispositivo online");
            return false;
        }

        device->enableMessagePolling(); //Habilita a coleta de mensagens no dispositivo
        device->addMessageCallback(std::make_shared<icsneo::MessageCallback>(
            [this](std::shared_ptr<icsneo::Message> message) {
                this->processMessage(message);
            }
        ));

        return true;
    }

private:
    shared_ptr<icsneo::Device> device;
    ros::Publisher can_pub;

    void processMessage(std::shared_ptr<icsneo::Message> msg) {
        // Verifica se a mensagem é do tipo CAN antes de processar
        if (msg->network.getType() == icsneo::Network::Type::CAN ||
            msg->network.getType() == icsneo::Network::Type::SWCAN ||
            msg->network.getType() == icsneo::Network::Type::LSFTCAN) {
    
            auto canmsg = static_pointer_cast<icsneo::CANMessage>(msg);
             // canmsg->arbid is valid here
            // canmsg->data is an std::vector<uint8_t>, you can check .size() for the DLC of the message
            // canmsg->timestamp is the time recorded by the hardware in nanoseconds since (1/1/2007 12:00:00 GMT)
            
            // Converte os dados binários diretamente para uma std::string
            std::string binary_data(reinterpret_cast<const char*>(canmsg->data.data()), canmsg->data.size());
            
            // Publica a mensagem no ROS
            std_msgs::String ros_msg;
            ros_msg.data = binary_data;
            can_pub.publish(ros_msg);
        }
    }
    

    void closeDevice() {
        if (device) {
            device->close();
            ROS_INFO("Dispositivo fechado com sucesso.");
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_listener");
    ros::NodeHandle nh;

    CANReader canReader(nh);
    if (!canReader.initialize()) {
        return 1;
    }

    // Mantém o nó ROS ativo
    ros::spin();

    return 0;
}
