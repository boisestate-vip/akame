
# Contributing

This file attempts to give a description of how contributions
to this project are expected to occur, as well as a general
standard for work contributed.

## Workflow

This will likely become a large project that has many people
contributing to it at the same time. Because of this, it is
important to follow certain practices to avoid the occurance
of [merge conflicts](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/about-merge-conflicts).

The following workflow will be expected for contributions:

 1. branch from main
 2. develop in branch
 3. create pull request to main
 4. review requested changes
 5. merge to main

Each step will be explained below.

### Branch From Main

The main branch in this repository will be composed
of the most up-to-date functioning versions of all
software. In order to preserve this and avoid cluttering
the work history, individual projects will be developed
in branches other than the main branch.

If you are unfamiliar with git branches, please refer
to the [linked tutorial](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell).

### Develop In Branch

This stage is self explanatory. After creating the branch,
perform all development work and changes inside of it.

### Create Pull Request to Main

Once development of the additional features are complete,
they must be added into the main branch in order for
users or other developers to be able to interact with them.
This can be done via a pull request. Pull requests
must be created to merge into main in this repository.

if you are unfamiliar with pull requests, please refer
to the [linked tutorial](https://github.blog/developer-skills/github/beginners-guide-to-github-creating-a-pull-request/).

### Review Requested Changes

Once a pull request is created, the code changes should
be reviewed for any issues or potential changes that
should not be included in the main branch.

I will note at this stage that there is no requirement
for code review in this repository, mainly due to reasons
of efficiency. However, if you are implementing code that
interacts with another developers's program or making changes to
another developer's code, it may be a good idea to request
them for review and wait until they have signed off to submit
your work.

### Merge to Main

Once the pull request has been reviewed, it can be merged to
main. When merging, it is preferred that you use the
``squash and merge`` option, as it will package all your
commits into a single element and prevent the commit log
from becoming too clogged up.

## Organization

This repository is organized in a way that is meant to
speed up navigation and reduce issues with group development.
Because of this, there are some expectations on the code
submitted to maintain the organization of the repository.

### Contribution Placement

You are expected to be aware of the directory structure and
place the work you submit in the appropriate location. ROS2
packages should not be located outside of the src directory,
and scripts should not be showing up in the doc directory.
There may of course be exceptions to this rule, however most
things will not be.

### Updating Documentation

Each subdirectory contains a README. If a contribution is made,
the relevant README should be updated to note this change. This
does not have to be done in a branch, and can be executed via
a quick push to main to avoid potential merge conflicts.

#### Updating index.md

It is important that anything in the doc directory
be referenced in index.md. This is because doc will likely get
very full as time goes on, so keeping a up-to-date map of what
each file is supposed to talk about will become important.

## Code Expectations

This section lists general expectaions for the code submitted
to the project.

### Encapsulation

Attempt to encapsulate the system you are working on to the
fullest extent possible. There will of course be code that
can and should be shared between programs, however it is much
eaiser to ensure that you will not overwrite someone else's
work if you stay in a single directory or file.

### Documentation

I do not personally have an issue with code that is not commented,
but I do have an issue with code that has no associated documentation.

Every subdirectory (e.g. package or program) should have its own
``README.md`` file in the top level of its directory structure. I do
not expect a README in every directory though. This file should at a
minimum explain what the program is supposed to do, any dependencies,
how to build it, and how to run it.

Any ROS2 package should contain in its README a seperate listing for every
node describing its inputs, outputs, and the operations it performs. It
should also contain descriptions of the launch files avaliable and any
custom messages produced.

Scripts should have a small description of their function and any assumptions
they make placed as a comment at the top of the script file.

### Code Quality

I normally wait to merge branches until I have at least a basic system working.
I place no real expectations on anyone else to do this though. As long as
sufficient documentation is provided describing the behavior and what works
and does not work, go ahead.

As for the structure and design of code and algorithms, it is expected that
they run in a reasonable amount of time, where reasonable is defined as within
the boundary needed for the system to meet its time constraints, as well
as fast enough that other systems do not stall waiting for it to complete.
Make an effort to produce readable code, as it will help you and others in
the future. I don't place any restrictions on what you do in your code though
as long as it works.

I will note that while there is little restriction on what code you decide
to contribute to this project, you will be seen as responsible for any code
you submit. This means that you will be asked questions about how it functions,
may be expected to fix any errors that occur in your systems, and could be called on
to make improvements or extensions to your work. Keep that in mind I guess :/.

## Submodules

If you would like to do most of your development in an external repository but
still link it to the main project (this repository), I would reccommend using
the git submodule feature. You can read about it in [this link](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
Follow the normal rules given above when using these, and you will be fine.

I would reccommend using the submodule system for your comptributions if
you are working on something you feel could be useful for talking to
potential employers or other similar things. You get to keep the
repository in your name and list it on your personal github page, and
you have more freedom with what do do with it after the project ends.
You also don't have to worry about merge conflicts as much, which is
pretty nice.

## Motivation

This section attempts to provide some justification for the annoying amount
of rules and red tape given above :).

Yes, a lot of this is kind of annoying and tedious. No, you will not be shamed
if you do not follow these rules exactly. They are here to serve as a reference
for anyone contributing or viewing the code. They have the goal of producing a
repository that can be used and referenced in the future, and of allowing
the programmers using it to build without making a mess.

Lots of companies in industry end up pretty harsh about how to go about
contributing to repositories, so you can treat this is an opportunity to
get used to working in a collaborative codebase. Once you start working
you will find that most of these expectations are pretty easy to follow.

Documentation is annoying, but it ends up being very helpful. Please take
your time with it and write it the way you would hope somebody else would
write documentation for you. The more documentation you write, the more
work it looks like you did, and the more people like working with you :).

## Future Directions

New sections may be added in the future and existing segments may be
updated. Any modifications will be documented below.

## Changelog

### 9/23/2025

- qeftser: constructed initial file
