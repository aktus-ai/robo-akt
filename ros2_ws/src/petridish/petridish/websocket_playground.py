

import time
import socketio
import asyncio


async def get_culture_type():
    async with socketio.AsyncSimpleClient() as sio:
        await sio.connect('http://localhost:3001')
        
        await sio.emit('join_room', 'accenture')
        await sio.emit('from_robot', 'get_culture_type')
        print(await sio.receive())


if __name__ == '__main__':
    asyncio.run(get_culture_type())
    print('done')
