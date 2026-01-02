MAVLink-Based Dual-Link Medium Switching for Reliable UAV Communication
Overview

This repository presents a Proof of Concept (PoC) for a reliable UAV communication framework designed for Search and Rescue (SAR) and disaster-response scenarios where conventional communication infrastructure is unavailable or unreliable.

The PoC implements dynamic medium switching between Wi-Fi and LoRa for MAVLink-based UAV communication using an application-layer Interface Manager running on a companion computer.
The objective is to ensure continuous command-and-control (C2) and telemetry delivery, even under link degradation or failure.

Problem Statement

UAVs deployed in heacy crowd zones face severe communication challenges:

Cellular and internet infrastructure is often too busy

Single-link communication systems fail due to:

Limited range (Wi-Fi)

Low data rate (LoRa)

Loss of control or telemetry can cause mission failure or safety hazards

Most UAV systems rely on one communication medium, making them vulnerable to single-point failures.

Proposed Solution :-

This PoC implements a heterogeneous dual-link communication architecture using:

MAVLink as the communication protocol

Wi-Fi for high-throughput, short-range data

LoRa for long-range, robust low-bandwidth communication

An Interface Manager (IM) dynamically selects the optimal medium based on real-time link quality metrics and ensures redundancy for critical messages.

System Architecture :-

Key Components :-

Flight Controller (FC)

Runs the autopilot stack

Produces and consumes MAVLink messages

Connected to companion computer via serial interface

Companion Computer (CC)

Runs Linux (Raspberry Pi / Jetson)

Hosts the Interface Manager

Routes MAVLink messages over Wi-Fi and/or LoRa

Dual Communication Interfaces

Wi-Fi (IEEE 802.11n)
High throughput, low latency, limited range

LoRa (ISM band)
Long range, low data rate, high robustness

Ground Control Station (GCS)

Receives MAVLink messages

Performs deduplication

Displays telemetry and allows command input

- Why MAVLink + Dual Links?

MAVLink defines how UAV data is structured, sequenced, and verified

Wi-Fi and LoRa define how data is physically transmitted

MAVLink alone does not guarantee delivery

Dual-link redundancy ensures reliability under unpredictable conditions

Medium Switching Concept :-

The core contribution of this PoC is application-layer medium switching.

Instead of relying on network-level failover, the system makes message-aware decisions based on real-time link conditions.

Interface Manager (IM) :-

The Interface Manager is the central logic unit of the PoC.

Responsibilities

Monitor link quality metrics

Decide which medium to use for each message

Duplicate critical messages across both links

Trigger failover without message loss

Link Quality Metrics Used

The IM continuously monitors:

Metric	Description :-
RSSI	Signal strength of the interface
Packet Loss Rate	Derived from MAVLink sequence numbers
HEARTBEAT Timeout	Missed MAVLink HEARTBEAT messages
Throughput	Estimated data transmission rate
Switching Logic

Under normal conditions:

Wi-Fi is preferred for telemetry and high-data traffic

When Wi-Fi quality degrades:

Packet loss increases

HEARTBEATs are missed

RSSI drops below threshold
→ IM switches primary transmission to LoRa

When Wi-Fi recovers:

IM dynamically reverts to Wi-Fi

Switching is automatic, adaptive, and continuous.

Message Priority and Redundancy

Message Classification :-

Critical Messages :-

C2 commands

HEARTBEAT

Position and health telemetry

Non-Critical Messages :-

Logs

High-rate telemetry

Video streams (out of scope for LoRa)

Redundancy Strategy

Critical messages are duplicated across both Wi-Fi and LoRa

Non-critical messages use the currently preferred interface only

Deduplication at GCS

Since critical messages may arrive twice:

Each MAVLink packet is hashed

Recently received hashes are cached

Duplicate packets are discarded

This ensures:

No repeated command execution

Idempotent message processing

Maximum delivery reliability

Implementation Details
Language & Tools

Python 3

pymavlink

Linux networking utilities

Mission Planner / QGroundControl

Key Modules :-

- interface_manager.py
Core switching and decision logic

- metrics_monitor.py
RSSI, packet loss, HEARTBEAT tracking

- mavlink_router.py
MAVLink packet handling and routing

- link_wifi.py
Wi-Fi communication abstraction

- link_lora.py
LoRa communication abstraction

- deduplicator.py
Duplicate packet detection

Demonstration Methodology :-

The project can be demonstrated using:

SITL or real flight controller

MAVLink telemetry over Wi-Fi

Artificial Wi-Fi degradation using Linux tc

Automatic switch to LoRa

Log verification of switching events

Continuous reception of critical telemetry

Observed Outcomes :-

Seamless switching between Wi-Fi and LoRa

No loss of critical MAVLink commands

Deterministic failover behavior

Improved reliability compared to single-link systems

Scope and Limitations :-

-Video streaming over LoRa is not supported

-Encryption/authentication is out of scope for this PoC

-Multi-UAV mesh networking is future work

-Focus is on command and telemetry reliability

Future Enhancements :-

-AI/ML-based predictive link switching

-Secure MAVLink transport

-Multi-UAV mesh integration

-Adaptive telemetry rate control

-Relevance to Capacity Building in UAS

This Project demonstrates:

-UAV communication system design

-Companion computer integration

-Reliability engineering

-Real-world network resilience

-Safety-critical system behavior

-It is well-suited for training, evaluation, and applied research in UAS technologies.

References :-

MAVLink Documentation: https://mavlink.io

PX4 Companion Computer Architecture

LoRa Alliance Technical Specifications

NS-3 Network Simulator

Research on MAVLink packet duplication and redundancy

Author

Kaushal Nilesh Chaudhari
Pune Institute of Computer Technology
Email: knc262005@gmail.com
