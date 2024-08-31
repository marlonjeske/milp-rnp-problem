[![DOI](https://zenodo.org/badge/850379107.svg)](https://zenodo.org/doi/10.5281/zenodo.13623490)

# A Mathematical Programming Approach to the Relay Node Placement Problem in Urban Environments


We propose a mixed-integer linear programming (MILP) model that optimizes relay node placement by minimizing installation costs, energy consumption, and communication delay while enhancing signal quality and ensuring network
fault tolerance.


## How to use

Run the `milp.py` file. 

It expects an input file in the same directory. You can use the `U1.txt` to `U4.txt` instance files.

The model result is outputed to a `log.txt` file


## Instance file

The instance input file has the number of sensors, number of relays, installation cost of each repeater, egde list initial energy of nodes, packet size and number of disjoint paths.

```
sensors <number of sensors>
relays <number of relays>
cost <relay ID> <cost>
...
e <from node> <to node> <edge weight> <edge capacity> <energy cost to transmit data> <energy cost to receive data>
...
initial_energy <node ID> <energy in joules>
...
packet_size <packet size in bits>
num_disjoints <number of disjoint paths>
```

Note: the nodes ID must follow the sequence: 1 is sink, 2 to S are sensor nodes and the rest are relays.


## Credits

Authors: Marlon Jeske, Valério Rosset, Daniel Aloise, Mariá C. V. Nascimento

