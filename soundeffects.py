import pygame
from random import choice
from time import sleep


music_path = "sounds/imperial_march.wav"

exit_sound_path = "sounds/have-the-protocol-droid's-mind-wiped.wav"

sound_paths = {
    'you underestimate my power': 'sounds/you-underestimate-my-power!.wav',
    'jedi scum': 'sounds/jedi-scum-rww.wav',
    'execute order 66': "sounds/execute-order-66.wav",
    'bring balance to the force': 'sounds/you-were-the-chosen-one-full.wav',
    'i was expecting': 'sounds/i-was-expecting-someone-with-your-reputation-to-be-a-little-older.wav',
    'i have the high ground': "sounds/it's-over-anakin!-i-have-the-high-ground!.wav",
    'power unlimited power': 'sounds/power!-unlimited-power!-tzw.wav',
    'you were the chosen one': "sounds/you-were-the-chosen-one-full.wav",
    'the republic will be reorganised': "sounds/the-republic-will-be-reorganised.wav",
    'you stupid little astro droid': "sounds/you-stupid-little-astro-droid.wav",
    'my new empire': "sounds/i-have-brought-peace-freedom-justice-and-security-to-my-new-empire!.wav",
    'back away i will deal with this jedi slime myself': "sounds/back-away!-i-will-deal-with-this-jedi-slime-myself.wav",
    'at last the jedi are no more': "sounds/at-last-the-jedi-are-no-more.wav",
    'only a sith deals in absolutes': "sounds/only-a-sith-deals-in-absolutes.wav",
    'so uncivilized': "sounds/so-uncivilized-ew5.wav"
}


def init_music(music_path=music_path):
    print(f'Music initialized at path "{music_path}"')
    pygame.mixer.music.load(music_path)

def toggle_music(is_paused):
    if is_paused:
        pygame.mixer.music.play(-1)
        is_paused = False
    else:
        pygame.mixer.music.stop()
        is_paused = True
    
    return is_paused

def init_sounds(*args):
    sounds = {}
    
    for _sounds in args:
        if type(_sounds) == dict:
            for key, value in _sounds.items():
                print(f'--> Sound named "{key}" initialized at path "{value}"')
                sound = pygame.mixer.Sound(value)
                sounds[key] = sound
        elif type(_sounds) == str:
            print(f'--> Sound at path {_sounds} initialized')
            yield pygame.mixer.Sound(_sounds)
    
    yield sounds


def play_sound(sound, channel_id=None, wait=False):
    if channel_id is not None:
        pygame.mixer.Channel(channel_id).play(sound)
    else:
        pygame.mixer.Sound.play(sound)

    if wait:
        length = pygame.mixer.Sound.get_length(sound)
        sleep(length)


def play_random_sound(sounds, channel_id=None):
    sound = choice(list(sounds.values()))
    play_sound(sound, channel_id)



if __name__ == "__main__":
    pygame.init()
    pygame.mixer.init()
    exit_sound, sounds = list(init_sounds(sound_paths, exit_sound_path))

    play_sound(sounds['the republic will be reorganised'], channel_id=0)
    sleep(2)
    play_sound(exit_sound, channel_id=0)

    input('Press Enter to end the program...')