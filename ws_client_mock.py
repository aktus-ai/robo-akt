
import sys
import random
import time
from typing import List

import socketio
import asyncio


async def get_culture_type():
    async with socketio.AsyncSimpleClient() as sio:
        print("connect:")
        await sio.connect('https://websocket-dot-aktus-393100.uw.r.appspot.com') 
        print("emit join room:") 
        await sio.emit('join_room', 'accenture')
        print("emit from robot")
        await sio.emit('from_robot', {'message': 'get_culture_type', 'room': 'accenture'})
        received = False
        culture_type = 'undefined'
        while not received:
                message = await sio.receive()
                print(message[1])
                if message[1]['event'] == 'from_model' and 'culture' in message[1]['data']['message']:
                        received = True
                        culture_type = message[1]['data']['message']['culture']
        print("await receive:")
        #from_model_message = await sio.receive()
        # TODO(maryam): Make sure the first element of the list is 'model_message'
        #print(from_model_message[1])
        #culture_type = from_model_message[1]['message']['culture']
        return culture_type



def main():
    print('Hi from petridish.')
    for i in range(50):
        wait_time = random.randint(3, 8)
        print('Sleeping for {} seconds'.format(wait_time))
        time.sleep(wait_time)
        culture_type = asyncio.run(get_culture_type())
        print('Received culture type of {}'.format(culture_type))


if __name__ == '__main__':
    main()