# Scenario
Conference submission system. Author submits, program chair "accept" or "reject".

# Gaia Design

## Environmental Model
Defines shared resources and constraints in the system

**Resources**
-  `SubmissionInbox`: a communication channel where submitted papers arrive.
- `ClassificationOutbox`: where classification decisions are posted

**Global Constraints**:
- Each submission must be classified within 24 hours.

## Role Model
- `Author`: submits paper to `SubmissionInbox`.
- `Chair`: retrieves submissions, classifies them, and responds to author.

### Protocols
- **Submit Paper**:
    - `author` -> `submissionInbox`: `submit(paperID, title)`
- **ClassifyPaper**:
    - `Chair` -> `submissionInbox`: `fetchNext()`
    - `chair` -> `author`: `classification(paperID, decision)`

## Interaction Model 
author -> submit(paper, title) -> chair
chair -> classification(paperID, decision)

## Agent Model
Maps abstract roles to concrete agent types; defers internal state

| role |Agent Class|
|---|---|
|author| `AuthorAgent`|
|Chair| `ChairAgent`|

> No state variables listed here, we defer those to implementation

## Service Model 
- **submitPaper**:
    - Provider: `AuthorAgent`
    - Input: `title: String`
    - Effect: places `submit(paperID, title)` in `SubmissionInbox`
- **classifyNext**:
    - Provider: `ChairAgent`
    - Input: `none`
    - Effect: places `decision: {accept, reject}` returned to `AuthorAgent`.
    
