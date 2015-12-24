#!/bin/bash

# Usage:
## 1 Modify the string of the variables TRIGGERED_COMMAND and then run the
##   the following commands to trigger the commands every 5 minutes.
## 2. crontab -e
## 3.  */5 * * * * path/to/opencog/docker/clone/buildbot/cronjobs.sh ~

# Environment Variables
## Name of script.
SELF_NAME=$(basename $0)

## Path to a repo clone
CLONE_PATH=""

## Path from which the script is invoked
CWD=$(pwd)

## File name where the output of this script is logged to. It would be in the
## same directory from which the script is invoked.
LOG_FILE="$SELF_NAME.log"

# Functions
## Check if the given repo has been cloned to the `cronjobs` directory. If it
## hasn't then makes a shallow clone from github into `cronjobs` directory.
## $1 : github opencog repository name. For eg. ocpkg for opencog/ocpkg repo.
set_workspace(){
    CLONE_PATH=$WORKSPACE/cronjobs/$1
    REPO_URL=https://github.com/opencog/$1.git

    if [ -d $CLONE_PATH ]; then
        printf "Changing directory to %s \n" "$CLONE_PATH"
        cd $CLONE_PATH

        if [ "$(git rev-parse --is-inside-work-tree)" == true ] ; then
            printf "%s contains a git repository \n" "$CLONE_PATH"

            # Just b/c it is named as the repo doesn't mean it has the given
            # repo
            REMOTE_ORIGIN="https:$(git remote show origin \
                                | grep -i "Fetch URL" | cut -d ":" -f 3)"

            if [ $REMOTE_ORIGIN == $REPO_URL ]; then
                printf "%s already cloned to %s \n" "$REPO_URL" "$CLONE_PATH"
            else
                printf "The repository in %s is not from %s \n" \
                    "$CLONE_PATH" "$REPO_URL"
            fi

        else
            printf "%s does not contain a git repository \n" "$CLONE_PATH"
            cd -
            rm -rf $CLONE_PATH
            printf "cloning %s to %s \n" "$1" "$CLONE_PATH"
            git clone --depth 1 $REPO_URL $CLONE_PATH
            cd $CWD
        fi

    else
        printf "cloning %s to %s \n" "$1" "$CLONE_PATH"
        git clone --depth 1 $REPO_URL $CLONE_PATH
        cd $CWD
    fi
}

## Check if the given repo's remote master has changed. If it has run the given
## command.
## $1 : github opencog repository name. For eg. ocpkg for opencog/ocpkg repo.
## $2 : a string of the command to be triggered for the repo.
trigger_command(){
    # If the workspace haven't been set, set it
    set_workspace $1
    cd $CLONE_PATH

    #REPO_NAME="$(basename \"$(git rev-parse --show-toplevel)\")"
    # fetch origin/upstream depending on the repo being dealt with
    git fetch origin
    ORIGIN_MASTER_HEAD=$(git log -1 --pretty="%H" origin/master)

    # Only check the state of the master branch
    git stash; git checkout master
    CURRENT_HEAD=$(git log -1 --pretty="%H")
    if [ $ORIGIN_MASTER_HEAD != $CURRENT_HEAD ] ; then

        # Trigger the command
        eval $2

        # Log every trigger
        printf "
        ********************************************************
        %s repository: triggered on orgin/master commit-hash = %s
        ******************************************************** \n" \
               "$1" "$ORIGIN_MASTER_HEAD"
        # update the origin
        git pull origin
    else
        printf "Did nothing b/c their hasn't been any change to %s repo \n" "$1"
    fi

    printf ">>>> %s repository: completed \n\n\n" "$1"
    cd $CWD
}

# Main Execution
## Redirect all stdout and stderr outputs to the $LOG_FILE.
exec &>>$LOG_FILE

printf "%s [%s] Starting ----------------------\n" "$(date)" "$SELF_NAME"

## Convert the argument passed to an abslute path
cd $1
WORKSPACE=$(pwd)
cd $CWD

## prevent the scripts from running unless a path is passed to it.
if [ -z $1 ]; then
    printf "No argument passed \n"
    cd $CWD
    printf "%s [%s] Finished ----------------------\n" "$(date)" "$SELF_NAME"
    exit 1
fi

## check if the directory given fits for a workspace. If it isn't a git
## repostiroy then it fits.
cd $WORKSPACE
if [ "$(git rev-parse --is-inside-work-tree)" == true ] ; then
    cd $WORKSPACE
    printf "%s contains a git repository. Use another directory \n" "$(pwd)"
    cd $CWD
    printf "%s [%s] Finished ----------------------\n" "$(date)" "$SELF_NAME"
    exit 1
fi

## Command to be run when condition are met. Modify it to suit your case.
## For updating docker images go to the `Build Settings` tab on hub.docker.com
## and use one of the curl `Build Trigger` options listed in the examples.
## For opencog/opencog-deps docker image an example trigger command is,
## curl -H "Content-Type: application/json" --data '{"build": true}' -X POST https://registry.hub.docker.com/u/opencog/opencog-deps/trigger/45dfbf0e-9412-4c6b-b3fd-a864e92ee9f6/

## For opencog/opencog-deps docker image
TRIGGERED_COMMAND='echo replace with the command for ocpkg repo'
trigger_command ocpkg "$TRIGGERED_COMMAND"

## For opencog/cogutils docker image
TRIGGERED_COMMAND="echo replace with the command for cogutils repo"
trigger_command cogutils "$TRIGGERED_COMMAND"

## For opencog/opencog-dev:cli docker image
TRIGGERED_COMMAND="echo replace with the command for atomspace repo"
trigger_command atomspace "$TRIGGERED_COMMAND"

printf "%s [%s] Finished ----------------------\n\n" "$(date)" "$SELF_NAME"
exit 0
