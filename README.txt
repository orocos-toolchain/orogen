= Orogen: easy component building for Orocos/RTT

* http://doudou.github.com/orogen
* http://github.com/doudou/orogen
* git://github.com/doudou/orogen.git

== What is oroGen ?

oroGen is a specification language and code generator for the Orocos Realtime
Toolkit (RTT). The Orocos/Real Time Toolkit is a C++ library allowing to build
component-based systems (http://www.orocos.org/rtt). The orogen tool allows to
create new components easily.

In oroGen, one write a specification that describes the components you want to
develop. Then, orogen generates the corresponding C++ code and CMake build
system so that you -- the component developer -- has only to care about
implementing the actual functionality.

The remaining of this readme is about oroGen installation. See the user's guide
for more in-depth usage information:

  http://doudou.github.com/orogen

== Dependencies

Orogen depends on the following libraries:
 - util.rb, a Ruby toolkit. It is available as a gem.
 - the Typelib library. This a C++ library with Ruby bindings.
 - util--, a C++ toolkit.

Generated modules of course depend on the Orocos Realtime Toolkit (RTT). Having
enabled CORBA support in the RTT is optional. Moreover, they depend on the
following libraries:
 - obviously the RTT itself. oroGen bases itself on an unreleased version of the
   RTT, which is available only on my github account. See below for details.
 - typelib, which is already a dependency of orogen itself

== Installing pre-packaged dependencies

The Typelib and Util-- libraries are available as Debian packages. Simply add
the following line to <tt>/etc/apt/sources.list</tt>:
 
  deb http://rubotics.rubyforge.org/debian ./

Installing them is then done with
  
  apt-get update
  apt-get install libtypelib1-ruby1.8 libutilmm1-dev

Util.rb is available as a RubyGem. See http://rubygems.org for information about
the RubyGems system. You can install it with

  gem install utilrb

== Installing dependencies from source

You can alternatively install all these dependencies directly from source.
Download the corresponding releases, or the latest bleeding edge, by taking them
from the following github account:

  http://www.github.com/doudou

Each package has its own INSTALL or README file explaining what to do.

== Installing the Orocos/RTT

oroGen bases itself on an unreleased version of the Orocos RTT. For now, it is
not the to-be 2.0 version that is being cooked by the project, but will migrate
to that when possible. The current version that works with oroGen is available
on github, as the new_data_flow branch of

  http://www.github.com/doudou/orocos-rtt
  http://www.github.com/doudou/orocos-ocl

