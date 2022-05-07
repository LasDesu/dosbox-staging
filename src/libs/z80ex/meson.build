extra_flags = []
if host_machine.endian() == 'big'
  extra_flags += '-DENDIANNESS=WORDS_BIG_ENDIAN'
else
  extra_flags += '-DENDIANNESS=WORDS_LITTLE_ENDIAN'
endif
extra_flags += '-DZ80EX_API_REVISION=1'
extra_flags += '-DZ80EX_VERSION_MAJOR=1'
extra_flags += '-DZ80EX_VERSION_MINOR=18'
extra_flags += '-DZ80EX_RELEASE_TYPE='
extra_flags += '-DZ80EX_VERSION_STR=1.1.18'

z80ex = static_library('z80ex', ['z80ex.c'],
                       include_directories : 'include',
                       c_args : extra_flags)

z80ex_dep = declare_dependency(link_with : z80ex,
                               include_directories : 'include')
