/* Generated from orogen/lib/orogen/templates/typekit/TypesDeprecated.hpp */
#pragma once

#warning including <%= typekit.name %>/Types.hpp directly is deprecated \
         You should include the headers that define your types directly. \
         typekit/Opaques.hpp should include <%= typekit.name %>/typekit/OpaqueFwd.hpp and \
         typekit/Opaques.cpp should include <%= typekit.name %>/typekit/OpaqueTypes.hpp. \
         (in doubt, see the files in templates/typekit for both of these) \
         Finally, if you are reall really sure, you can still include \
         <%= typekit.name %>/typekit/Types.hpp \
         Be warned that this last option may disappear at any moment
#include <<%= typekit.name %>/typekit/Types.hpp>

