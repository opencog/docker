#!/bin/bash

# Usage:
#  ./cronjobs.sh $HOME/path/to/dir/to/workspace

# Environment Variables
## Name of script.
SELF_NAME=$(basename $0)

## Path from which the script is invoked
CWD=$(pwd)

## Command to be run when condition are met. Modify it to suit your case.
## For updating docker images go to the `Build Settings` tab on hub.docker.com
## and use one of the curl `Build Trigger` options listed in the examples.
TRIGGERED_COMMAND="echo triggered command not set"
        #curl -H "Content-Type: application/json" --data '{"build": true}' -X POST https://registry.hub.docker.com/u/opencog/opencog-deps/trigger/45dfbf0e-9412-4c6b-b3fd-a864e92ee9f6/

## File name where the output of this script is logged to. It would be in the
## same directory.
LOG_FILE=my.log

#exec 2>&1 1>z.log


# Functions
## Check if the given repo has been cloned to the `cronjobs` directory. If it
## hasn't then makes a shallow clone from github into `cronjobs` directory.
## $1 : github opencog repository name. For eg. ocpkg for opencog/ocpkg repo.
set_workspace(){
    DIR_PATH=$WORKSPACE/cronjobs/$1
    REPO_URL=https://github.com/opencog/$1.git

    if [ -d $DIR_PATH ]; then
        printf "Changing directory to %s \n" "$DIR_PATH"
        cd $DIR_PATH

        if [ "$(git rev-parse --is-inside-work-tree)" == true ] ; then
            printf "%s contains a git repository \n" "$DIR_PATH"

            # Just b/c it is named as the repo doesn't mean it has the given
            # repo
            REMOTE_ORIGIN="https:$(git remote show origin \
                                | grep -i "Fetch URL" | cut -d ":" -f 3)"
            echo $REMOTE_ORIGIN
            echo $REPO_URL
            if [ $REMOTE_ORIGIN == $REPO_URL ]; then
                printf "%s already cloned to %s \n" "$REPO_URL" "$DIR_PATH"
            else
                printf "The repository in %s is not from %s \n" \
                    "$DIR_PATH" "$REPO_URL"
            fi
        else
            printf "%s does not contain a git repository \n" "$DIR_PATH"
            cd -
            rm -rf $DIR_PATH
            printf "cloning %s to %s \n" "$1" "$DIR_PATH"
            git clone --depth 1 $REPO_URL $DIR_PATH
            cd $CWD
        fi

    else
        printf "cloning %s to %s \n" "$1" "$DIR_PATH"
        git clone --depth 1 $REPO_URL $DIR_PATH
        cd $CWD
    fi
}

# Main Execution
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

## For opencog/opencog-deps docker image
TRIGGERED_COMMAND="echo ocpkg-trigger-replace-it"

# For opencog/cogutils docker image
TRIGGERED_COMMAND="echo ocpkg-trigger-replace-it"
set_workspace ocpkg
printf "%s [%s] Finished ----------------------\n" "$(date)" "$SELF_NAME"
