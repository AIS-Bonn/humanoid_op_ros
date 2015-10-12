Coding Guidelines  {#coding}
=================

[TOC]

Code repository
---------------

We use a central `git` repository for our project. Here is a small `git`
guide to get you started:

* Cloning the repository: see [Installation](@ref installation).
* Pulling commits from the repository:
  ~~~~{.sh}
  git pull --rebase
  ~~~~
  The `--rebase` flag avoids creating merge commits, which result in non-linear
  history. So always use it for straight pulls.
  If you need to merge something (git will warn you with a big *CONFLICT*
  message), edit the mentioned files and do an `git add` on the files. Then
  use `git rebase --continue` to continue the process.
* Committing changes
  ~~~~{.sh}
  git gui
  ~~~~
  This will pop up a nice GUI for selecting files (or even lines) to include
  in your commit. Select the files from the left by clicking on the **icons**
  next to the files.
  Type a short message describing your changes. The format should be something
  like this:
  ~~~~{.sh}
  hardware/robotcontrol: higher publish frequency for joints
  
  Gives us more precise information about what is going on.
  ~~~~
  The first line should form a short description about what you have done.
  Not more than one line (80 chars) here! If you want to include more
  information, do that in the second part, separated by a newline.
* Pushing your commits to the central repository
  ~~~~{.sh}
  git push origin master
  ~~~~
  git will complain if someone else also pushed changes since your last pull.
  Just do a new pull (see above) and try again.

Coding style
------------
General tips:

* Use tabs for indentation. And **only** for indentation. We use a tab size of 4
  spaces in our editors, but this is not important. The code should look
  like you want it to with any tab size setting. This means you should use
  tabs to get to the current indentation level. Further alignment (e.g. if you
  are doing a long method call or aligning some table) should be done with
  spaces.
* No whitespace at the end of the line. Most editors enforce this by default.
  `git gui` will warn you with big red markers if you try to commit whitespace
  errors.
* Every source file should start with a one-line comment describing what it
  does. You can use identical comments in corresponding header and
  implementation files. The second line should contain the main author of the
  file.
  Example:
~~~~{.h}
// Walking gait based on the capture step framework
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
~~~~

* Capitalization: SomeClass and SomeClass::someMethod().
* Member variables should begin with the prefix `m_`.
* Move your libraries into namespaces.

More basic C++ tips:

* Parameter passing. If you pass something the callee should modify, use a
  *pointer*. If the argument should not be modified, use a *constant reference*.
  In code:
~~~~{.cpp}
// Declaration:
void someMethod(const Struct1& constant, Struct2* variable);

// And the call:
Struct1 c;
Struct2 v;
someMethod(c, &v);
~~~~
  You can immediately tell from the call which struct is going to be modified.
  Do not pass large arguments simply by value!
* Use `boost::shared_ptr<>` if you save pointers in multiple locations. That
  will save you a lot of anguish.
