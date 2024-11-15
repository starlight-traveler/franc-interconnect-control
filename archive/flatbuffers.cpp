#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "flatbuffers/flatbuffers.h"
#include "example_generated.h"

void setup()
{
    Serial.begin(9600);

    flatbuffers::FlatBufferBuilder builder(1024);

    // Create weapon names
    auto weapon_one_name = builder.CreateString("Sword");
    auto weapon_two_name = builder.CreateString("Axe");

    // Create weapons
    auto sword = MyGame::Sample::CreateWeapon(builder, weapon_one_name, 3);
    auto axe = MyGame::Sample::CreateWeapon(builder, weapon_two_name, 5);

    // Create a vector of weapons
    std::vector<flatbuffers::Offset<MyGame::Sample::Weapon>> weapons_vector = {sword, axe};
    auto weapons = builder.CreateVector(weapons_vector);

    // Create name
    auto name = builder.CreateString("Orc");

    // Create position struct
    MyGame::Sample::Vec3 pos(1.0f, 2.0f, 3.0f);

    // Create the Monster
    auto orc = MyGame::Sample::CreateMonster(
        builder,
        &pos,
        150, // mana
        80,  // hp
        name,
        0, // inventory
        MyGame::Sample::Color_Red,
        weapons,
        MyGame::Sample::Equipment_Weapon,
        axe.Union(),
        0 // path
    );

    // Finish the buffer
    builder.Finish(orc);

    // Get a pointer to the root object inside the buffer
    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    // Print the size of the buffer
    Serial.print("FlatBuffer size: ");
    Serial.println(size);

    // Deserialize the buffer
    auto monster = MyGame::Sample::GetMonster(buf);

    // Access the monster's data
    Serial.print("Monster Name: ");
    Serial.println(monster->name()->c_str());
    Serial.print("Monster HP: ");
    Serial.println(monster->hp());
}

void loop()
{
    // Your code here
}
