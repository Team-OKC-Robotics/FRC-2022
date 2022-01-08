# Software goals
 - **not be the reason we lose**
 - be robust
 - be helpful
 - be fast

## Not be the reason we lose
This is the main objective of the programming division. Like, the main main objective. It's fine if a mechanical problem, a driver problem, or even just a strategy problem is the reason we lose. We might not lose at all. But it is **not** going to be programming's fault if we do.

## Be robust
This ties in with not being the reason we lose. The code needs to be able to work and run no matter what. Connection crappy and we keep dropping packets? Keep sending them until you get an ACK. Motor came unplugged? Don't crash. Camera not seeing anything? Have a backup. *No matter what don't fail*. Part of this is just adding null checks and whatnot to the code. Another part is having a set of core functions that always have to work, no matter the cost, so we're never left dead on the field.

## Be helpful
The code needs to help. Part of #1 "not be the reason we lose" is doing no harm. But more than that the code needs to help. To what degree it does is up to the drivers, but it needs to make things easy and obvious for humans.

## Be fast
This also ties into objective #1, as the code can't not lose if it's running too slow. Of course, it is more important that it runs at all than to be running fast, but still, the code needs to be relatively quick. It needs to be *responsive*. This is last on the goal list, however, and "premature optimization is the root of all evil." So this is the last goal to turn to, after everything above has been taken care of.

# Guiding principles
To aid in the successful achievement of the above goals, there are some guiding principles behind the development of this software.
 1. make it work
 2. make it work right
 3. make it readable
 4. make it work fast

And of course, a little KISS and Zen of Python never hurt.

> Simple is better than complex  
> Complex is better than complicated  
> Special cases aren't special enough to break the rules  
> Although practicality beats purity