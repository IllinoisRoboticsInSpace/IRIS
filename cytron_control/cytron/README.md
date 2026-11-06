#### Linear Actuators for Scoop

| Cytron Pin | Arduino Nano R4 Pin | Description |
|---|---|---|
| GND | GND | Common ground |
| PWM2 | D3 | Motor 2 PWM (~) |
| DIR2 | D2 | Motor 2 direction |
| PWM1 | D5 | Motor 1 PWM (~) |
| DIR1 | D4 | Motor 1 direction |

#### Linear Actuators for BAB

| Cytron Pin | Arduino Pin | Description |
|---|---|---|
| GND | GND | Common ground |
| PWM2 | D6 | Motor 2 PWM (~) |
| DIR2 | D7 | Motor 2 direction |
| PWM1 | D9 | Motor 1 PWM (~) |
| DIR1 | D8 | Motor 1 direction |

#### Limit Switch

| Limit Switch Terminal | Arduino Nano R4 Pin | Description |
|---|---|---|
| NC | 5V | Normally closed contact |
| COM | A0 | Common contact / digital input |

#### Diagram

```mermaid
flowchart LR
    subgraph NANO["Arduino Nano R4"]
        direction TB
        GND["GND"]
        D2["D2"]
        D3["D3 ~"]
        D4["D4"]
        D5["D5 ~"]
        D6["D6 ~"]
        D7["D7"]
        D8["D8"]
        D9["D9 ~"]
        A0["A0"]
        V5["5V"]
    end

    subgraph SCOOP["Cytron — Scoop"]
        direction TB
        SG["GND"]
        SP2["PWM2"]
        SD2["DIR2"]
        SP1["PWM1"]
        SD1["DIR1"]
    end

    subgraph BAB["Cytron — BAB"]
        direction TB
        BG["GND"]
        BP2["PWM2"]
        BD2["DIR2"]
        BP1["PWM1"]
        BD1["DIR1"]
    end

    subgraph LIMIT["Limit Switch"]
        direction TB
        NC["NC"]
        COM["COM"]
    end

    GND --- SG
    GND --- BG

    D3 --> SP2
    D2 --> SD2
    D5 --> SP1
    D4 --> SD1

    D6 --> BP2
    D7 --> BD2
    D9 --> BP1
    D8 --> BD1

    V5 --- NC
    A0 --- COM

    classDef arduino fill:#e0f2fe,stroke:#0284c7,color:#075985
    classDef motor fill:#dcfce7,stroke:#16a34a,color:#166534
    classDef limit fill:#fef3c7,stroke:#d97706,color:#92400e

    class GND,D2,D3,D4,D5,D6,D7,D8,D9,A0,V5 arduino
    class SG,SP2,SD2,SP1,SD1,BG,BP2,BD2,BP1,BD1 motor
    class NC,COM limit
```