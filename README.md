# 2021-group-10

## To execute the code 
 - git clone git@git.chalmers.se:courses/dit638/students/2021-group-10.git
 - cd 2021-group-10/src
 - docker build -t cid/example:latest -f Dockerfile .
 - docker run --rm cid/example:latest &lt;a number&gt;


## Team workflow 
- To add new features: 
    - each member will create a separate branch to implement that feature 
    - when completed, each member creates a merge request asking another member to review the changes 
    - Once approved, the branch can be merged into master
  ----
- To fix features:
    - a member will create a separate branch to work on a fix 
    - once done, the member creates a merge request asking another member to review the changes
    - Once approved, the branch can be merged into master

## Rules for commiting messages 
- We will ensure to follow the below rules for commit messages:
- Add a title and a body separated from each other
- Start the message with capital letter
- Do not end the message with a . / ? / !
- Use imperative language
- The body of message should describe the ‘what’ not the ‘how’
- The message should be descriptive (addressing the added/updated features in the commit)
- No more than 70 characters in one commit message


----
## Tools required: 
- git
- Ubuntu
- Docker
