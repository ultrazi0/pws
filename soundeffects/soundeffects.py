import pygame
from random import choice
from time import sleep


music_path = "sounds/starwars/imperial_march.wav"

exit_sound_path = "sounds/starwars/have-the-protocol-droid's-mind-wiped.wav"

sound_paths_starwars = {
    'you underestimate my power': 'sounds/starwars/you-underestimate-my-power!.wav',
    'jedi scum': 'sounds/starwars/jedi-scum-rww.wav',
    'execute order 66': "sounds/starwars/execute-order-66.wav",
    'bring balance to the force': 'sounds/starwars/you-were-the-chosen-one-full.wav',
    'i was expecting': 'sounds/starwars/i-was-expecting-someone-with-your-reputation-to-be-a-little-older.wav',
    'i have the high ground': "sounds/starwars/it's-over-anakin!-i-have-the-high-ground!.wav",
    'power unlimited power': 'sounds/starwars/power!-unlimited-power!-tzw.wav',
    'you were the chosen one': "sounds/starwars/you-were-the-chosen-one-full.wav",
    'the republic will be reorganised': "sounds/starwars/the-republic-will-be-reorganised.wav",
    'you stupid little astro droid': "sounds/starwars/you-stupid-little-astro-droid.wav",
    'my new empire': "sounds/starwars/i-have-brought-peace-freedom-justice-and-security-to-my-new-empire!.wav",
    'back away i will deal with this jedi slime myself': "sounds/starwars/back-away!-i-will-deal-with-this-jedi-slime-myself.wav",
    'at last the jedi are no more': "sounds/starwars/at-last-the-jedi-are-no-more.wav",
    'only a sith deals in absolutes': "sounds/starwars/only-a-sith-deals-in-absolutes.wav",
    'so uncivilized': "sounds/starwars/so-uncivilized-ew5.wav"
}


sound_paths_aim = {
     'identify target': "sounds/starcraft/identify-target.wav",
     'nuclear missle ready': "sounds/starcraft/nuclear-missle-ready.wav",
     'not enough energy': "sounds/starcraft/not-enough-energy.wav"
}

sound_paths_shoot = {
     'yes sir': "sounds/starcraft/yes-sir.wav",
     'orders sir': "sounds/starcraft/orders-sir.wav",
     'tank setting up': "sounds/starcraft/tank-setting-up.wav",
     'justice': "sounds/starcraft/and-dispense-some-indiscriminate-justice.wav",
     'tta-ta-ta-da': "sounds/starcraft/tta-ta-ta-da.wav"
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


def init_one_sound(sound: str):
    assert type(sound) == str

    print(f'--> Sound at path "{sound}" initialized')
    
    return pygame.mixer.Sound(sound)


def init_sounds(sounds: dict):
    assert type(sounds) == dict

    _sounds = {}

    for key, value in sounds.items():
                print(f'--> Sound named "{key}" initialized at path "{value}"')
                sound = pygame.mixer.Sound(value)
                _sounds[key] = sound
    
    return _sounds


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


def play_random_sound_from_selection(sounds: dict, selection: list, channel_id=None, wait=False):
    assert type(selection) == list, "Parameter 'selection' must be a list"

    for sound in selection:
          assert sound in sounds.keys(), f'Sound "{sound}" not in dictionary'
    
    sound = choice(selection)
    play_sound(sounds[sound], channel_id, wait)


def stop_sound_on_channel(channel_id=0):
    pygame.mixer.Channel(channel_id).stop()
    

if __name__ == "__main__":
    pygame.init()
    pygame.mixer.init()
    init_music()
    sounds_aim = init_sounds(sound_paths_aim)
    sounds_shoot = init_sounds(sound_paths_shoot)

    play_random_sound_from_selection(sounds_shoot, channel_id=0, wait=True, selection=['tta-ta-ta-da', 'justice'])

    input('Press Enter to end the program...')