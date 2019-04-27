/**
 * OpenSpirulina http://www.openspirulina.com
 *
 * Autors: Sergio Arroyo (UOC)
 * 
 * OpenSpirulina Shield connections configuration
 * 
 */
#ifndef OpenSpir_SHIELD_Conn_h
#define OpenSpir_SHIELD_Conn_h

//===========================================================
//===================== Shield connectors ===================
//===========================================================
#define OPENSPIR_SHIELD_J1         PIN_A10                 // Analog 10
#define OPENSPIR_SHIELD_J2         PIN_A11                 // Analog 11
#define OPENSPIR_SHIELD_J3         PIN_A12                 // Analog 12
#define OPENSPIR_SHIELD_J4         PIN_A13                 // Analog 13 - Current sensor 1
#define OPENSPIR_SHIELD_J5         PIN_A14                 // Analog 14 - Current sensor 2
#define OPENSPIR_SHIELD_J6         PIN_A15                 // Analog 15

#define OPENSPIR_SHIELD_SW1        43                      // Switch 1 calibration

#define OPENSPIR_VGA_PIN1          24                      // Red led
#define OPENSPIR_VGA_PIN2          26                      // Blue led
//#define OPENSPIR_VGA_PIN3                                // Free
#define OPENSPIR_VGA_PIN4          32                      // DHT22 sensor
//#define OPENSPIR_VGA_PIN5                                // GND
#define OPENSPIR_VGA_PIN6          28                      // Green led
#define OPENSPIR_VGA_PIN7          34                      // ADDR for external lux BH1750 sensor
#define OPENSPIR_VGA_PIN8                                  // Free
//#define OPENSPIR_VGA_PIN9                                // GND
#define OPENSPIR_VGA_PIN10         PIN_WIRE_SDA            // SDA
#define OPENSPIR_VGA_PIN11         PIN_WIRE_SCL            // SCL
//#define OPENSPIR_VGA_PIN12                               // Vcc (+)
//#define OPENSPIR_VGA_PIN13                               // Vcc (+)
#define OPENSPIR_VGA_PIN14         30                      // Temperature OneWire
//#define OPENSPIR_VGA_PIN15                               // Free

#endif
