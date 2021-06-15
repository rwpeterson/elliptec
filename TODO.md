# Client-server architecture for experiment control

## async primitives: serial

Serial is very slow, and the equipment it controls is often slower.

This is a perfect match for async concurrency.

### Example: waveplate control

Several experiments are wall-clock limited by the time to sequentially
manipulate waveplates.

Solution: Send serial commands to multiple waveplate controllers
concurrently, with "hot" ones like tomography waveplates with 1-1
dedicated controllers to complete frequent operations in one hardware
cycle, and less-used waveplates muxed on controllers and ran
sequentially when needed.

The server managing the serial controllers accepts an RPC with the desired
state/update, determines which serial commands need to be sent, and sends
them to work queues for each controller. The allocation of the waveplates
to controllers is transparent at point of use, although clearly can be
optimized to minimize time.

TODO: can the controller to motor protocol be reverse-engineered to use
e.g. one arduino per motor for maximum parallelism?

## RPC model

There are several client-server communication schemes, from RPC to more
web-focused ones like REST or OpenAPI, that may be more specific in the
transport layer, etc. that they use.

As we use it for data serialization already, Cap'n Proto RPC is a
natural choice for us. Unlike self-describing formats like JSON,
serialized RPC information requires a schema be used at the client and
server. This specifies the API, and the design of the protocol allows for
degrees of forwards and backwards compatibility following certain rules
for modifying the schema, removing a lot of manual labor in maintaining
client and server code as protocols evolve.

## Sketch of waveplate example

```
+--------------------------------+
|         Client example         |
+---------------+----------------+
| Waveplate API | Powermeter API |
+---------------+----------------+
         ^
         | Cap'n Proto RPC
         v
+---+---------+-----------+   \
|   | RPC API |           |    \
|   +---------+           |     \
|        |                |     /
|        +-- System state |    /
|        |                |    \
|    Serial API           |     \
|        |                |      \ Waveplate
|    Job queues           |      / server
|   /    |    \           |     /
|                         |    /
|S|     |S|     |S|       |    \
|E|     |E|     |E|       |     \
|R| ... |R| ... |R|       |     /
|1|     |i|     |N|       |    /
+-+-----+-+-----+-+-------+   /
         ^
         | async serial library
         v
    +---------+
    |OS Serial|
    |  API    |
    +---------+
         ^
         | serial over USB
         v
    +------------+
    | Controller |
    +------------+
      ^        ^
      |  ...   | ??? protocol
      v        v
    Motor1   MotorF

```

What's new for me:

- async serial library (pain point for windows support?)
- async server
- async job control
- Cap'n Proto RPC
- async client example

What language?

- Rust would be nice but even the non-async serial library is looking for
  new maintiner, and the async ones are on ancient tokio verions, etc.
- Python's libraries have their own issues: `pyserial`'s `pyserial-asyncio`
  does not support Windows! `aioserial` a better option? Seems to just be
  pyserial where it runs in a different thread. Ok? Better than writing it
  all myself, and it looks pretty minimal.

Changes to `elliptec`?

- Perhaps factor out the serial device creation to avoid supporting whatever
  arbitrary junk you need to initialize the connection? But for this specific
  device, these are already mostly fixed.
- Testing setup with `socat(1)` rather than my hacky `Dummyio` class?
  See above for class initialization design.
- Format of the async features? Separate class? Can I separate the
  dependencies or have it as a feature?
- capnp schema
- reference python client
